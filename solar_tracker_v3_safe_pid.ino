/*
  =============================================================================
  DUAL AXIS SOLAR TRACKER V3 - SAFE SINGLE-5V + FULL 180-DEG PID (ESP32)
  =============================================================================
  Compatible with Arduino-ESP32 Core 2.x and 3.x (LEDC hardware PWM).

  CRITICAL WIRING SAFETY (Single 5V Power Rail):
    1. Power ESP32 via USB (5V) OR regulated 5V into VIN/5V pin.
    2. 4x LDR dividers MUST use ESP32 3.3V pin (NOT 5V!):
         3.3V -- LDR -- ADC pin (GPIO 34,35,32,33) -- 10k resistor -- GND
       (Connecting LDRs to 5V can exceed 3.3V max on ADC pins and damage ESP32!)
    3. 2x SG90 Servos connect to 5V/VIN pin + common GND:
         PAN  signal -> GPIO 18
         TILT signal -> GPIO 19
       Recommended: place a 470uF-1000uF electrolytic capacitor across 5V & GND
       near the servo power lines to absorb current spikes.

  IMPROVEMENTS & PROTECTIONS IN V3:
    - Full 0-180 deg Pan travel (520-2480us pulse range; avoids 1000-2000us 90-deg cap)
    - Velocity PID controller with anti-windup, D-term low-pass filter, D-clamp,
      continuous deadband, and D-kick reset guard
    - Single-5V rail protection:
        * Sequential startup (Pan attaches/moves first, then Tilt)
        * Acceleration slew-rate ramp (cuts servo inrush current spikes)
        * Combined 2-axis speed budget (caps simultaneous current draw)
        * Idle PWM release (duty=0 after settling in DEADBAND/LOW_LIGHT/FAULT
          to prevent continuous holding/end-stop stall current & overheating)
    - Sensor fault guard + optional Serial calibration ('c'/'r') & range test ('0'/'1'/'h'/'a')
  =============================================================================
*/

#include <Arduino.h>
#include <math.h>
#include <esp_arduino_version.h>

// ----------------------------------------------------------------------------
// 1. PIN & HARDWARE PWM CONFIGURATION
// ----------------------------------------------------------------------------
constexpr uint8_t LDR_PINS[4] = {34, 35, 32, 33}; // TL, TR, BL, BR (ADC1 only)
constexpr uint8_t PAN_PIN = 18;
constexpr uint8_t TILT_PIN = 19;
constexpr uint8_t PAN_CHANNEL = 0;
constexpr uint8_t TILT_CHANNEL = 1;

constexpr uint32_t PWM_FREQ = 50;
constexpr uint8_t PWM_BITS = 13;
constexpr float PWM_PERIOD_US = 20000.0f;
constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_BITS) - 1;

// ----------------------------------------------------------------------------
// 2. MECHANICAL LIMITS & SG90 PULSE CALIBRATION
// ----------------------------------------------------------------------------
constexpr float PAN_HOME = 90.0f;
constexpr float TILT_HOME = 90.0f;

constexpr float PAN_MIN = 0.0f;      // Full 180-deg horizontal tracking
constexpr float PAN_MAX = 180.0f;
constexpr float TILT_MIN = 15.0f;    // Safety margin to prevent frame collision
constexpr float TILT_MAX = 165.0f;

// Standard SG90 full-range pulse: ~500-2500us.
// 520-2480us gives full ~180 deg while keeping a tiny buffer off hard stops
// so the servo gear train does not stall at 700mA+. Trim if your SG90 buzzes.
constexpr float PULSE_MIN_US = 520.0f;
constexpr float PULSE_MAX_US = 2480.0f;

// Invert sign (+1 <-> -1) if an axis rotates AWAY from the light source.
constexpr float PAN_DIRECTION = -1.0f;
constexpr float TILT_DIRECTION = +1.0f;

// ----------------------------------------------------------------------------
// 3. SINGLE-5V CURRENT / BROWNOUT PROTECTION LIMITS
// ----------------------------------------------------------------------------
// Single axis max speed: 35 deg/s -> 180 deg sweep takes ~5.2s (fast enough for
// phone flashlight demo, gentle enough for a single 5V rail).
constexpr float MAX_SINGLE_SPEED_DEG_S = 35.0f;

// Total combined speed budget across Pan + Tilt at the same instant.
// Prevents both SG90s from drawing peak stall current simultaneously.
constexpr float MAX_COMBINED_SPEED_DEG_S = 40.0f;

// Acceleration slew limit (deg/s^2): smooths speed changes so servos do not
// slam from 0 to max speed in one tick (major source of 5V voltage dips).
constexpr float MAX_ACCEL_DEG_S2 = 120.0f;

// Stop sending 50Hz PWM pulses (duty=0) after holding steady for 2s in
// DEADBAND / LOW_LIGHT / FAULT / MANUAL to keep servos cool and 5V rail safe.
// Set to 0 to disable idle release if a heavy panel sags on Tilt.
constexpr uint32_t IDLE_RELEASE_MS = 2000;

// ----------------------------------------------------------------------------
// 4. SENSOR FILTERING & PID TUNING
// ----------------------------------------------------------------------------
constexpr uint8_t MEDIAN_N = 5;
constexpr uint16_t ADC_SAMPLE_GAP_US = 150;
constexpr float EMA_ALPHA = 0.25f;

constexpr float DEADBAND = 0.03f;       // 3% normalized light difference
constexpr float LIGHT_OFF = 500.0f;     // Hysteresis low threshold (sum of 4 ADC)
constexpr float LIGHT_ON = 750.0f;      // Hysteresis high threshold

// PID velocity gains (output = target angular velocity in deg/s)
constexpr float KP_PAN = 125.0f;
constexpr float KI_PAN = 8.0f;
constexpr float KD_PAN = 4.5f;

constexpr float KP_TILT = 105.0f;
constexpr float KI_TILT = 6.0f;
constexpr float KD_TILT = 4.0f;

constexpr float D_FILTER_ALPHA = 0.30f;
constexpr float D_TERM_MAX = 22.0f;
constexpr float INTEGRAL_LIMIT = 0.8f;

constexpr uint32_t CONTROL_MS = 20;     // 50 Hz loop matched to 50 Hz servo PWM
constexpr uint32_t SERIAL_MS = 200;

// Fault & saturation diagnostics
constexpr bool ENABLE_SATURATION_GUARD = true;
constexpr uint16_t SATURATED_ADC = 4090;
constexpr float OTHER_THREE_MAX_SUM = 350.0f;
constexpr uint32_t FAULT_CONFIRM_MS = 2000;

// ----------------------------------------------------------------------------
// 5. STATE VARIABLES
// ----------------------------------------------------------------------------
struct PIDController {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float dFiltered;
  bool primed;
};

PIDController pidPan  = {KP_PAN,  KI_PAN,  KD_PAN,  0.0f, 0.0f, 0.0f, false};
PIDController pidTilt = {KP_TILT, KI_TILT, KD_TILT, 0.0f, 0.0f, 0.0f, false};

float raw[4]      = {0.0f, 0.0f, 0.0f, 0.0f};
float filtered[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float gain[4]     = {1.0f, 1.0f, 1.0f, 1.0f};

float panAngle  = PAN_HOME;
float tiltAngle = TILT_HOME;
float manualPanTarget  = PAN_HOME;
float manualTiltTarget = TILT_HOME;
float errPan    = 0.0f;
float errTilt   = 0.0f;
float panSpeed  = 0.0f;
float tiltSpeed = 0.0f;

bool tracking = false;
bool sensorFault = false;
bool manualHold = false;
bool servosReleased = false;

uint32_t suspiciousSince[4] = {0, 0, 0, 0};
uint32_t lastControl = 0;
uint32_t lastSerial = 0;
uint32_t lastActiveMs = 0;

// ----------------------------------------------------------------------------
// 6. SENSOR ACQUISITION & CALIBRATION
// ----------------------------------------------------------------------------
uint16_t medianRead(uint8_t pin) {
  uint16_t a[MEDIAN_N];
  for (uint8_t i = 0; i < MEDIAN_N; ++i) {
    a[i] = analogRead(pin);
    delayMicroseconds(ADC_SAMPLE_GAP_US);
  }
  for (uint8_t i = 1; i < MEDIAN_N; ++i) {
    uint16_t key = a[i];
    int j = (int)i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      --j;
    }
    a[j + 1] = key;
  }
  return a[MEDIAN_N / 2];
}

void sampleSensors(bool initialize = false) {
  for (int i = 0; i < 4; ++i) {
    raw[i] = (float)medianRead(LDR_PINS[i]) * gain[i];
    filtered[i] = initialize
                    ? raw[i]
                    : (EMA_ALPHA * raw[i] + (1.0f - EMA_ALPHA) * filtered[i]);
  }
}

void calibrateGainsNow() {
  float sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  constexpr int SAMPLES = 32;
  for (int s = 0; s < SAMPLES; ++s) {
    for (int i = 0; i < 4; ++i) {
      sum[i] += (float)medianRead(LDR_PINS[i]);
    }
    delay(5);
  }
  float mean = (sum[0] + sum[1] + sum[2] + sum[3]) / 4.0f;
  if (mean / SAMPLES < LIGHT_ON / 4.0f) {
    Serial.println("# CALIBRATION SKIPPED: Not enough light (keep 4 LDRs evenly lit)");
    return;
  }
  for (int i = 0; i < 4; ++i) {
    gain[i] = (sum[i] > 1.0f) ? constrain(mean / sum[i], 0.5f, 2.0f) : 1.0f;
  }
  sampleSensors(true);
  Serial.printf("# CALIBRATED gains: TL=%.2f TR=%.2f BL=%.2f BR=%.2f\n",
                gain[0], gain[1], gain[2], gain[3]);
}

// ----------------------------------------------------------------------------
// 7. LEDC SERVO CONTROL & SINGLE-RAIL SAFE OUTPUT
// ----------------------------------------------------------------------------
bool initPanPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(PAN_PIN, PWM_FREQ, PWM_BITS);
#else
  double f = ledcSetup(PAN_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PAN_PIN, PAN_CHANNEL);
  return f > 0;
#endif
}

bool initTiltPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(TILT_PIN, PWM_FREQ, PWM_BITS);
#else
  double f = ledcSetup(TILT_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(TILT_PIN, TILT_CHANNEL);
  return f > 0;
#endif
}

void writeServo(uint8_t pin, uint8_t channel, float angle,
                float minAngle, float maxAngle) {
  angle = constrain(angle, minAngle, maxAngle);
  float pulse = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * angle / 180.0f;
  uint32_t duty = (uint32_t)lroundf((pulse / PWM_PERIOD_US) * PWM_MAX_DUTY);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void releaseServo(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, 0);
#else
  (void)pin;
  ledcWrite(channel, 0);
#endif
}

void outputServos() {
  servosReleased = false;
  writeServo(PAN_PIN, PAN_CHANNEL, panAngle, PAN_MIN, PAN_MAX);
  writeServo(TILT_PIN, TILT_CHANNEL, tiltAngle, TILT_MIN, TILT_MAX);
}

void releaseServos() {
  if (!servosReleased) {
    releaseServo(PAN_PIN, PAN_CHANNEL);
    releaseServo(TILT_PIN, TILT_CHANNEL);
    servosReleased = true;
  }
}

// ----------------------------------------------------------------------------
// 8. PID CONTROLLER & CURRENT-SAFE SPEED RAMPING
// ----------------------------------------------------------------------------
void resetPid(PIDController &pid) {
  pid.integral = 0.0f;
  pid.prevError = 0.0f;
  pid.dFiltered = 0.0f;
  pid.primed = false;
}

float computePidVelocity(PIDController &pid, float error, float dt,
                         float angle, float minAngle, float maxAngle,
                         float direction) {
  if (!pid.primed) {
    pid.prevError = error;
    pid.dFiltered = 0.0f;
    pid.primed = true;
  }

  float rawD = (error - pid.prevError) / dt;
  pid.prevError = error;
  pid.dFiltered += D_FILTER_ALPHA * (rawD - pid.dFiltered);

  float absErr = fabsf(error);
  if (absErr <= DEADBAND) {
    pid.integral = 0.0f;
    return 0.0f;
  }

  float activeErr = copysignf(absErr - DEADBAND, error);
  float pTerm = pid.kp * activeErr;
  float dTerm = constrain(pid.kd * pid.dFiltered, -D_TERM_MAX, D_TERM_MAX);

  float nextIntegral = constrain(pid.integral + activeErr * dt,
                                 -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float candidate = pTerm + pid.ki * nextIntegral + dTerm;

  bool atMin = angle <= minAngle + 0.05f;
  bool atMax = angle >= maxAngle - 0.05f;
  float deltaAngle = direction * candidate;
  bool angleSat = (atMin && deltaAngle < 0.0f) || (atMax && deltaAngle > 0.0f);
  bool speedSat = (candidate > MAX_SINGLE_SPEED_DEG_S && activeErr > 0.0f) ||
                  (candidate < -MAX_SINGLE_SPEED_DEG_S && activeErr < 0.0f);

  if (!(angleSat || speedSat) || (activeErr * pid.integral < 0.0f)) {
    pid.integral = nextIntegral;
  }

  float output = pTerm + pid.ki * pid.integral + dTerm;
  return constrain(output, -MAX_SINGLE_SPEED_DEG_S, MAX_SINGLE_SPEED_DEG_S);
}

float applySlewLimit(float currentSpeed, float targetSpeed, float maxStep) {
  float diff = targetSpeed - currentSpeed;
  if (diff > maxStep)  return currentSpeed + maxStep;
  if (diff < -maxStep) return currentSpeed - maxStep;
  return targetSpeed;
}

// ----------------------------------------------------------------------------
// 9. FAULT GUARD, DIAGNOSTICS & SERIAL COMMANDS
// ----------------------------------------------------------------------------
bool detectFault(uint32_t now, float rawTotal) {
  if (!ENABLE_SATURATION_GUARD) return false;
  bool fault = false;
  for (int i = 0; i < 4; ++i) {
    bool suspicious = raw[i] >= SATURATED_ADC &&
                      (rawTotal - raw[i]) <= OTHER_THREE_MAX_SUM;
    if (suspicious) {
      if (suspiciousSince[i] == 0) suspiciousSince[i] = now;
      if (now - suspiciousSince[i] >= FAULT_CONFIRM_MS) fault = true;
    } else {
      suspiciousSince[i] = 0;
    }
  }
  return fault;
}

void printCsv(uint32_t now, const char *state) {
  // CSV: DATA,ms,TL,TR,BL,BR,errPan,errTilt,panDeg,tiltDeg,panDegPerS,tiltDegPerS,state
  Serial.printf("DATA,%lu,%.0f,%.0f,%.0f,%.0f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%s\n",
    (unsigned long)now, raw[0], raw[1], raw[2], raw[3],
    errPan, errTilt, panAngle, tiltAngle, panSpeed, tiltSpeed, state);
}

float stepToward(float current, float target, float maxStep) {
  float diff = target - current;
  if (fabsf(diff) <= maxStep) return target;
  return current + copysignf(maxStep, diff);
}

void handleSerialCommand(uint32_t now) {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == '0') {
      manualHold = true;
      manualPanTarget = PAN_MIN;
      manualTiltTarget = tiltAngle;
      panSpeed = tiltSpeed = 0.0f;
      lastActiveMs = now;
      resetPid(pidPan);
      resetPid(pidTilt);
      Serial.println("# MANUAL: Ramping Pan -> 0 deg (send 'a' for auto tracking)");
    } else if (cmd == '1') {
      manualHold = true;
      manualPanTarget = PAN_MAX;
      manualTiltTarget = tiltAngle;
      panSpeed = tiltSpeed = 0.0f;
      lastActiveMs = now;
      resetPid(pidPan);
      resetPid(pidTilt);
      Serial.println("# MANUAL: Ramping Pan -> 180 deg (send 'a' for auto tracking)");
    } else if (cmd == 'h' || cmd == 'H') {
      manualHold = true;
      manualPanTarget = PAN_HOME;
      manualTiltTarget = TILT_HOME;
      panSpeed = tiltSpeed = 0.0f;
      lastActiveMs = now;
      resetPid(pidPan);
      resetPid(pidTilt);
      Serial.println("# MANUAL: Ramping Home -> 90/90 deg (send 'a' for auto tracking)");
    } else if (cmd == 'a' || cmd == 'A') {
      manualHold = false;
      lastActiveMs = now;
      resetPid(pidPan);
      resetPid(pidTilt);
      outputServos();
      Serial.println("# AUTO: PID tracking resumed");
    } else if (cmd == 'c' || cmd == 'C') {
      calibrateGainsNow();
    } else if (cmd == 'r' || cmd == 'R') {
      for (int i = 0; i < 4; ++i) gain[i] = 1.0f;
      sampleSensors(true);
      Serial.println("# RESET: LDR gains restored to 1.0");
    }
  }
}

// ----------------------------------------------------------------------------
// 10. SETUP & MAIN LOOP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (uint8_t pin : LDR_PINS) pinMode(pin, INPUT);

  // Sequential startup: attach & settle Pan first, then Tilt, so both servos
  // never draw startup stall current on the 5V rail at the exact same instant.
  if (!initPanPwm()) {
    Serial.println("ERROR: Pan PWM initialization failed");
    while (true) delay(1000);
  }
  writeServo(PAN_PIN, PAN_CHANNEL, panAngle, PAN_MIN, PAN_MAX);
  delay(600);

  if (!initTiltPwm()) {
    Serial.println("ERROR: Tilt PWM initialization failed");
    while (true) delay(1000);
  }
  writeServo(TILT_PIN, TILT_CHANNEL, tiltAngle, TILT_MIN, TILT_MAX);
  delay(600);

  sampleSensors(true);
  lastControl = lastSerial = lastActiveMs = millis();

  Serial.println("# Dual Axis Solar Tracker V3 | Safe Single-5V + 180-deg PID");
  Serial.println("# WIRING CHECK: LDR VCC MUST be 3.3V! Servos on 5V + Common GND.");
  Serial.println("# Cmds: '0'=Pan0, '1'=Pan180, 'h'=Home, 'a'=Auto, 'c'=Calibrate, 'r'=ResetGain");
  Serial.println("# DATA,ms,TL,TR,BL,BR,errPan,errTilt,panDeg,tiltDeg,panDegPerS,tiltDegPerS,state");
}

void loop() {
  uint32_t now = millis();
  handleSerialCommand(now);

  if ((uint32_t)(now - lastControl) < CONTROL_MS) return;
  float dt = (uint32_t)(now - lastControl) / 1000.0f;
  lastControl = now;
  dt = constrain(dt, 0.001f, 0.10f);

  sampleSensors();
  float rawTotal = raw[0] + raw[1] + raw[2] + raw[3];
  float total = filtered[0] + filtered[1] + filtered[2] + filtered[3];
  sensorFault = detectFault(now, rawTotal);

  if (tracking) {
    if (rawTotal < LIGHT_OFF || total < LIGHT_OFF) tracking = false;
  } else {
    if (rawTotal >= LIGHT_ON && total >= LIGHT_ON) tracking = true;
  }

  errPan = 0.0f;
  errTilt = 0.0f;
  float targetPanSpeed = 0.0f;
  float targetTiltSpeed = 0.0f;
  const char *state = "LOW_LIGHT";
  bool activelyMoving = false;

  if (manualHold) {
    state = "MANUAL";
    float prevPan = panAngle;
    float prevTilt = tiltAngle;
    // Move one axis at a time in manual mode (Pan first, then Tilt) to keep peak current low
    float step = MAX_SINGLE_SPEED_DEG_S * dt;
    if (fabsf(manualPanTarget - panAngle) > 0.01f) {
      panAngle = stepToward(panAngle, manualPanTarget, step);
    } else if (fabsf(manualTiltTarget - tiltAngle) > 0.01f) {
      tiltAngle = stepToward(tiltAngle, manualTiltTarget, step);
    }
    panSpeed = (panAngle - prevPan) / dt;
    tiltSpeed = (tiltAngle - prevTilt) / dt;
    if (fabsf(panAngle - prevPan) > 0.001f || fabsf(tiltAngle - prevTilt) > 0.001f) {
      activelyMoving = true;
      lastActiveMs = now;
    }
  } else if (sensorFault) {
    state = "SENSOR_FAULT";
    panSpeed = 0.0f;
    tiltSpeed = 0.0f;
    resetPid(pidPan);
    resetPid(pidTilt);
  } else if (tracking && total > 0.0f) {
    float left = filtered[0] + filtered[2];
    float right = filtered[1] + filtered[3];
    float top = filtered[0] + filtered[1];
    float bottom = filtered[2] + filtered[3];

    errPan = (left - right) / total;
    errTilt = (top - bottom) / total;

    targetPanSpeed = computePidVelocity(
      pidPan, errPan, dt, panAngle, PAN_MIN, PAN_MAX, PAN_DIRECTION);
    targetTiltSpeed = computePidVelocity(
      pidTilt, errTilt, dt, tiltAngle, TILT_MIN, TILT_MAX, TILT_DIRECTION);

    // Combined 2-axis speed budget: scales down proportionally when both axes
    // demand high speed simultaneously to protect the single 5V supply.
    float combined = fabsf(targetPanSpeed) + fabsf(targetTiltSpeed);
    if (combined > MAX_COMBINED_SPEED_DEG_S && combined > 0.0f) {
      float scale = MAX_COMBINED_SPEED_DEG_S / combined;
      targetPanSpeed *= scale;
      targetTiltSpeed *= scale;
    }

    // Smooth acceleration ramp prevents sudden current surges
    float maxStep = MAX_ACCEL_DEG_S2 * dt;
    panSpeed = applySlewLimit(panSpeed, targetPanSpeed, maxStep);
    tiltSpeed = applySlewLimit(tiltSpeed, targetTiltSpeed, maxStep);

    // Stop cleanly inside deadband once ramped down
    if (targetPanSpeed == 0.0f && fabsf(panSpeed) < 0.5f) panSpeed = 0.0f;
    if (targetTiltSpeed == 0.0f && fabsf(tiltSpeed) < 0.5f) tiltSpeed = 0.0f;

    float prevPan = panAngle;
    float prevTilt = tiltAngle;
    panAngle = constrain(panAngle + PAN_DIRECTION * panSpeed * dt, PAN_MIN, PAN_MAX);
    tiltAngle = constrain(tiltAngle + TILT_DIRECTION * tiltSpeed * dt, TILT_MIN, TILT_MAX);

    // Zero velocity if pushed against mechanical bounds so reverse response is instant
    if ((panAngle <= PAN_MIN + 0.01f && PAN_DIRECTION * panSpeed < 0.0f) ||
        (panAngle >= PAN_MAX - 0.01f && PAN_DIRECTION * panSpeed > 0.0f)) {
      panSpeed = 0.0f;
    }
    if ((tiltAngle <= TILT_MIN + 0.01f && TILT_DIRECTION * tiltSpeed < 0.0f) ||
        (tiltAngle >= TILT_MAX - 0.01f && TILT_DIRECTION * tiltSpeed > 0.0f)) {
      tiltSpeed = 0.0f;
    }

    bool moved = (fabsf(panAngle - prevPan) > 0.001f) ||
                 (fabsf(tiltAngle - prevTilt) > 0.001f);

    if (moved) {
      activelyMoving = true;
      lastActiveMs = now;
      state = "TRACK";
    } else if (targetPanSpeed == 0.0f && targetTiltSpeed == 0.0f) {
      state = "DEADBAND";
    } else {
      // Commanded direction is clamped at 0/180 or Tilt mechanical boundary
      state = "LIMIT_HOLD";
    }
  } else {
    panSpeed = 0.0f;
    tiltSpeed = 0.0f;
    resetPid(pidPan);
    resetPid(pidTilt);
  }

  // Power & thermal guard: release servo PWM after settling idle so servos do
  // not continuously pull current fighting gear friction or mechanical stops.
  if (activelyMoving) {
    outputServos();
  } else if (IDLE_RELEASE_MS > 0 && (uint32_t)(now - lastActiveMs) >= IDLE_RELEASE_MS) {
    releaseServos();
  } else if (!servosReleased) {
    outputServos();
  }

  if ((uint32_t)(now - lastSerial) >= SERIAL_MS) {
    lastSerial = now;
    printCsv(now, state);
  }
}
