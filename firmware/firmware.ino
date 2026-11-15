/*
  DUAL AXIS SOLAR TRACKER V3 - ESP32 classic + 4 LDR + 2 SG90
  Target: Arduino-ESP32 Core 3.x (Core 2.x fallback kept). No ESP32Servo needed.

  Wiring: 3.3V -- LDR -- ADC pin -- 10k -- GND  (more light -> higher ADC)
    LDR  TL GPIO34, TR GPIO35, BL GPIO32, BR GPIO33
    PAN  GPIO18, TILT GPIO19 (LEDC 50 Hz, 13-bit), servos on external 5V, common GND.

  Control pipeline (per axis, every CONTROL_MS):
    ADC median-of-5 -> EMA -> normalized imbalance y in [-1, 1]
    -> Schmitt deadband (start > DB_HIGH, stop < DB_LOW)
    -> PID (P/I on deadband-shifted error, D on filtered measurement)
    -> velocity command [deg/s], clamped to MAX_SPEED
    -> acceleration ramp (MAX_ACCEL) -> per-cycle step clamp (MAX_STEP)
    -> angle -> LEDC duty (written only when the duty count changes)

  Modes: TRACK/HOLD (sun) -> HOMING (dark, confirmed) -> SLEEP (PWM off)
         SLEEP/HOMING -> TRACK when light returns (confirmed).

  Serial live tuning (115200, newline-terminated, case-insensitive):
    P<v> I<v> D<v>   PID gains (deg/s per unit normalized error)
    DB<v>            deadband HIGH (also sets LOW = HIGH/2), e.g. DB0.06
    DL<v>            deadband LOW,  e.g. DL0.03
    DT<v>            D-term low-pass time constant [s]
    A<v>             EMA alpha (0.01..1)
    S<v> C<v> X<v>   max speed [deg/s], max accel [deg/s^2], max step [deg/cycle]
    G<v>             tilt gain ratio (tilt gains = pan gains * G)
    L<v>             night threshold (average ADC of 4 LDRs)
    Q                toggle CSV stream     ?  print params     H  help
*/
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <esp_arduino_version.h>

// ---------------- Hardware ----------------
constexpr uint8_t LDR_PINS[4] = {34, 35, 32, 33}; // TL TR BL BR
constexpr uint8_t PAN_PIN = 18, TILT_PIN = 19;
constexpr uint8_t PAN_CHANNEL = 0, TILT_CHANNEL = 1; // Core 2.x only
constexpr uint32_t PWM_FREQ = 50;
constexpr uint8_t PWM_BITS = 13;
constexpr float PWM_PERIOD_US = 20000.0f;
constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_BITS) - 1;
constexpr float PULSE_MIN_US = 1000.0f, PULSE_MAX_US = 2000.0f;

// ---------------- Mechanics ----------------
constexpr float PAN_HOME = 90.0f, TILT_HOME = 90.0f;
constexpr float PAN_MIN = 20.0f, PAN_MAX = 160.0f;
constexpr float TILT_MIN = 20.0f, TILT_MAX = 160.0f;
// Verified on your rig in V2: keep these signs.
constexpr float PAN_DIRECTION = -1.0f;
constexpr float TILT_DIRECTION = +1.0f;

// ---------------- Timing / sampling ----------------
constexpr uint32_t CONTROL_MS = 20;        // matches the 50 Hz servo frame
constexpr uint32_t SERIAL_MS = 200;
constexpr uint8_t MEDIAN_N = 5;
constexpr uint16_t ADC_SAMPLE_GAP_US = 150;

// ---------------- Fixed control constants ----------------
constexpr float I_LIMIT_RATIO = 0.5f;      // |I term| <= ratio * maxSpeed
constexpr float HOME_KP = 1.5f;            // deg/s per deg of home error
constexpr float HOME_SPEED = 4.0f;         // deg/s, gentle return at dusk
constexpr float HOME_TOLERANCE = 0.3f;     // deg
constexpr float LIGHT_WAKE_RATIO = 1.5f;   // wake threshold = L * ratio
constexpr uint32_t DARK_CONFIRM_MS = 3000; // ignore passing clouds / hands
constexpr uint32_t LIGHT_CONFIRM_MS = 2000;

// ---------------- Saturation guard ----------------
constexpr bool ENABLE_SATURATION_GUARD = true;
constexpr uint16_t SATURATED_ADC = 4090;
constexpr float OTHER_THREE_MAX_SUM = 350.0f;
constexpr uint32_t FAULT_CONFIRM_MS = 2000;

// ---------------- Live-tunable parameters ----------------
struct Tuning {
  float kp;         // deg/s per unit error
  float ki;         // deg/s per (unit error * s)
  float kd;         // deg/s per (unit error / s)
  float dbHigh;     // start moving above this |error|
  float dbLow;      // stop moving below this |error|
  float dTau;       // s, D-term low-pass time constant
  float emaAlpha;
  float maxSpeed;   // deg/s
  float maxAccel;   // deg/s^2
  float maxStepDeg; // deg per control cycle
  float tiltRatio;  // tilt gains = pan gains * tiltRatio
  float minLight;   // average ADC (sum / 4) below which it is "night"
};

Tuning tune = {
  20.0f, 0.5f, 0.4f,
  0.06f, 0.03f,
  0.10f,
  0.20f,
  8.0f, 30.0f, 0.25f,
  0.75f,
  125.0f,
};

// ---------------- Axis state ----------------
struct Axis {
  uint8_t pin, channel;
  float minDeg, maxDeg, home, direction;
  float angle;     // commanded angle [deg]
  float vel;       // actual ramped velocity, angle frame [deg/s]
  float integ;     // integral contribution, error frame [deg/s]
  float prevMeas;
  float dFilt;     // filtered d(measurement)/dt
  float err;       // last normalized error (for logging)
  float cmd;       // last PID velocity command, error frame (for logging)
  bool moving;     // Schmitt deadband state
  bool hasPrev;
  uint32_t lastDuty;
};

Axis pan  = {PAN_PIN,  PAN_CHANNEL,  PAN_MIN,  PAN_MAX,  PAN_HOME,  PAN_DIRECTION,
             PAN_HOME,  0, 0, 0, 0, 0, 0, false, false, UINT32_MAX};
Axis tilt = {TILT_PIN, TILT_CHANNEL, TILT_MIN, TILT_MAX, TILT_HOME, TILT_DIRECTION,
             TILT_HOME, 0, 0, 0, 0, 0, 0, false, false, UINT32_MAX};

enum class Mode : uint8_t { TRACKING, HOMING, SLEEPING };
Mode mode = Mode::TRACKING;

float raw[4] = {0, 0, 0, 0};
float filtered[4] = {0, 0, 0, 0};
float gain[4] = {1, 1, 1, 1}; // calibrate only with controlled uniform light
uint32_t suspiciousSince[4] = {0, 0, 0, 0};
uint32_t lastControl = 0, lastSerial = 0;
uint32_t darkSince = 0, brightSince = 0;
bool csvEnabled = true;
const char *stateName = "INIT";

// =====================================================================
// Sensors
// =====================================================================
uint16_t medianRead(uint8_t pin) {
  uint16_t a[MEDIAN_N];
  for (uint8_t i = 0; i < MEDIAN_N; ++i) {
    a[i] = analogRead(pin);
    delayMicroseconds(ADC_SAMPLE_GAP_US);
  }
  for (uint8_t i = 1; i < MEDIAN_N; ++i) {
    uint16_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; --j; }
    a[j + 1] = key;
  }
  return a[MEDIAN_N / 2];
}

void sampleSensors(bool initialize = false) {
  for (int i = 0; i < 4; ++i) {
    raw[i] = medianRead(LDR_PINS[i]) * gain[i];
    filtered[i] = initialize ? raw[i]
                : tune.emaAlpha * raw[i] + (1.0f - tune.emaAlpha) * filtered[i];
  }
}

bool detectFault(uint32_t now, float rawTotal) {
  if (!ENABLE_SATURATION_GUARD) return false;
  bool fault = false;
  for (int i = 0; i < 4; ++i) {
    bool suspicious = raw[i] >= SATURATED_ADC &&
                      (rawTotal - raw[i]) <= OTHER_THREE_MAX_SUM;
    if (suspicious) {
      if (suspiciousSince[i] == 0) suspiciousSince[i] = now | 1; // 0 means "not set"
      if (now - suspiciousSince[i] >= FAULT_CONFIRM_MS) fault = true;
    } else {
      suspiciousSince[i] = 0;
    }
  }
  return fault;
}

// =====================================================================
// Servo output
// =====================================================================
bool initPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(PAN_PIN, PWM_FREQ, PWM_BITS) &&
         ledcAttach(TILT_PIN, PWM_FREQ, PWM_BITS);
#else
  double f1 = ledcSetup(PAN_CHANNEL, PWM_FREQ, PWM_BITS);
  double f2 = ledcSetup(TILT_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PAN_PIN, PAN_CHANNEL);
  ledcAttachPin(TILT_PIN, TILT_CHANNEL);
  return f1 > 0 && f2 > 0;
#endif
}

uint32_t angleToDuty(const Axis &ax, float angle) {
  angle = constrain(angle, ax.minDeg, ax.maxDeg);
  float pulse = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * angle / 180.0f;
  return (uint32_t)lroundf((pulse / PWM_PERIOD_US) * PWM_MAX_DUTY);
}

// Skipping identical duty writes avoids re-latching the LEDC every cycle.
void writeDuty(Axis &ax, uint32_t duty) {
  if (duty == ax.lastDuty) return;
  ax.lastDuty = duty;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ax.pin, duty);
#else
  ledcWrite(ax.channel, duty);
#endif
}

void outputAxis(Axis &ax) { writeDuty(ax, angleToDuty(ax, ax.angle)); }

// Duty 0 = no pulses: SG90 stops holding torque and draws only idle current.
void releaseAxis(Axis &ax) { writeDuty(ax, 0); }

// =====================================================================
// Control
// =====================================================================
void resetPid(Axis &ax) {
  ax.integ = 0.0f;
  ax.dFilt = 0.0f;
  ax.hasPrev = false;
  ax.moving = false;
  ax.err = 0.0f;
  ax.cmd = 0.0f;
}

bool atLimitPushingOut(const Axis &ax, float angleFrameVel) {
  return (angleFrameVel > 0.0f && ax.angle >= ax.maxDeg - 1e-3f) ||
         (angleFrameVel < 0.0f && ax.angle <= ax.minDeg + 1e-3f);
}

// meas: normalized imbalance (setpoint 0). Returns velocity command in the
// error frame [deg/s]; caller applies the axis direction sign.
float pidUpdate(Axis &ax, float meas, float gainScale, float dt) {
  ax.err = meas;
  float absE = fabsf(meas);

  // D on the filtered measurement, updated every cycle (even in HOLD) so it is
  // primed and does not kick when the deadband releases or the error is
  // reshaped by the deadband offset below.
  float dRaw = ax.hasPrev ? (meas - ax.prevMeas) / dt : 0.0f;
  ax.prevMeas = meas;
  ax.hasPrev = true;
  ax.dFilt += (dt / (tune.dTau + dt)) * (dRaw - ax.dFilt);

  // Schmitt-trigger deadband.
  if (!ax.moving && absE > tune.dbHigh) ax.moving = true;
  else if (ax.moving && absE < tune.dbLow) ax.moving = false;

  if (!ax.moving) {
    ax.integ = 0.0f; // bumpless re-entry
    ax.cmd = 0.0f;
    return 0.0f;
  }

  // Shift by DB_LOW so the command fades to exactly zero at the stop boundary.
  float e = copysignf(absE - tune.dbLow, meas);

  float kp = tune.kp * gainScale;
  float ki = tune.ki * gainScale;
  float kd = tune.kd * gainScale;

  float p = kp * e;
  float d = constrain(kd * ax.dFilt, -tune.maxSpeed, tune.maxSpeed);
  float unsat = p + ax.integ + d;

  // Anti-windup: conditional integration + hard clamp.
  bool pushingSaturation = (unsat >  tune.maxSpeed && e > 0.0f) ||
                           (unsat < -tune.maxSpeed && e < 0.0f);
  bool pushingLimit = atLimitPushingOut(ax, ax.direction * e);
  if (!pushingSaturation && !pushingLimit) {
    float iLim = I_LIMIT_RATIO * tune.maxSpeed;
    ax.integ = constrain(ax.integ + ki * e * dt, -iLim, iLim);
  }

  ax.cmd = constrain(p + ax.integ + d, -tune.maxSpeed, tune.maxSpeed);
  return ax.cmd;
}

// Velocity ramp (accel limit) + per-cycle step clamp, in the angle frame.
void moveAxis(Axis &ax, float targetVel, float dt) {
  float dv = tune.maxAccel * dt;
  ax.vel += constrain(targetVel - ax.vel, -dv, dv);

  float step = constrain(ax.vel * dt, -tune.maxStepDeg, tune.maxStepDeg);
  ax.vel = step / dt; // keep the ramp state consistent with the real motion

  float want = ax.angle + step;
  float next = constrain(want, ax.minDeg, ax.maxDeg);
  if (next != want) ax.vel = 0.0f; // hit a mechanical limit
  ax.angle = next;
}

// Returns true when the axis has settled at home.
bool homeAxis(Axis &ax, float dt) {
  float diff = ax.home - ax.angle;
  float target = constrain(HOME_KP * diff, -HOME_SPEED, HOME_SPEED);
  moveAxis(ax, target, dt);
  if (fabsf(ax.home - ax.angle) < HOME_TOLERANCE && fabsf(ax.vel) < 0.2f) {
    ax.angle = ax.home;
    ax.vel = 0.0f;
    return true;
  }
  return false;
}

// =====================================================================
// Serial live tuning
// =====================================================================
struct Param { const char *key; float *ptr; float lo, hi; };

// Longer keys first so "DB" is not parsed as "D".
const Param PARAMS[] = {
  {"DB", &tune.dbHigh,     0.0f,  0.5f},
  {"DL", &tune.dbLow,      0.0f,  0.5f},
  {"DT", &tune.dTau,       0.0f,  2.0f},
  {"P",  &tune.kp,         0.0f,  500.0f},
  {"I",  &tune.ki,         0.0f,  500.0f},
  {"D",  &tune.kd,         0.0f,  100.0f},
  {"A",  &tune.emaAlpha,   0.01f, 1.0f},
  {"S",  &tune.maxSpeed,   0.5f,  60.0f},
  {"C",  &tune.maxAccel,   1.0f,  1000.0f},
  {"X",  &tune.maxStepDeg, 0.01f, 5.0f},
  {"G",  &tune.tiltRatio,  0.1f,  3.0f},
  {"L",  &tune.minLight,   0.0f,  4095.0f},
};

void printParams() {
  Serial.printf("# P=%.4f I=%.4f D=%.4f DB=%.4f DL=%.4f DT=%.3f A=%.3f\n",
                tune.kp, tune.ki, tune.kd, tune.dbHigh, tune.dbLow,
                tune.dTau, tune.emaAlpha);
  Serial.printf("# S=%.2f C=%.2f X=%.3f G=%.3f L=%.1f CSV=%d\n",
                tune.maxSpeed, tune.maxAccel, tune.maxStepDeg,
                tune.tiltRatio, tune.minLight, csvEnabled ? 1 : 0);
}

void printHelp() {
  Serial.println("# Commands: P<v> I<v> D<v> DB<v> DL<v> DT<v> A<v> S<v> C<v> X<v> G<v> L<v>");
  Serial.println("#           Q=toggle CSV  ?=params  H=help   e.g. P18  DB0.05  A0.15");
}

void handleCommand(char *line) {
  // Uppercase and strip spaces in place.
  char *w = line;
  for (char *r = line; *r; ++r) {
    if (!isspace((unsigned char)*r)) *w++ = (char)toupper((unsigned char)*r);
  }
  *w = '\0';
  if (line[0] == '\0') return;

  if (strcmp(line, "?") == 0) { printParams(); return; }
  if (strcmp(line, "H") == 0) { printHelp(); return; }
  if (strcmp(line, "Q") == 0) {
    csvEnabled = !csvEnabled;
    Serial.printf("# CSV %s\n", csvEnabled ? "ON" : "OFF");
    return;
  }

  for (const Param &p : PARAMS) {
    size_t n = strlen(p.key);
    if (strncmp(line, p.key, n) != 0) continue;

    const char *numStr = line + n;
    char *end = nullptr;
    float v = strtof(numStr, &end);
    if (end == numStr || *end != '\0' || !isfinite(v) || v < p.lo || v > p.hi) {
      Serial.printf("# ERR %s: value must be %.3f..%.3f\n", p.key, p.lo, p.hi);
      return;
    }
    *p.ptr = v;

    // Keep DB_LOW <= DB_HIGH.
    if (p.ptr == &tune.dbHigh) tune.dbLow = tune.dbHigh * 0.5f;
    if (p.ptr == &tune.dbLow && tune.dbLow > tune.dbHigh) tune.dbLow = tune.dbHigh;
    if (p.ptr == &tune.ki) { pan.integ = 0.0f; tilt.integ = 0.0f; }

    Serial.printf("# OK %s=%.4f\n", p.key, v);
    return;
  }
  Serial.printf("# ERR unknown command '%s' (H for help)\n", line);
}

void pollSerial() {
  static char buf[32];
  static uint8_t len = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (!overflow && len > 0) {
        buf[len] = '\0';
        handleCommand(buf);
      }
      len = 0;
      overflow = false;
    } else if (overflow) {
      // drop until end of line
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      overflow = true;
      Serial.println("# ERR line too long");
    }
  }
}

// =====================================================================
// Logging
// =====================================================================
void printCsv(uint32_t now) {
  // DATA,ms,TL,TR,BL,BR,errPan,errTilt,panDeg,tiltDeg,panDegPerS,tiltDegPerS,state
  Serial.printf("DATA,%lu,%.0f,%.0f,%.0f,%.0f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%s\n",
    (unsigned long)now, raw[0], raw[1], raw[2], raw[3],
    pan.err, tilt.err, pan.angle, tilt.angle, pan.vel, tilt.vel, stateName);
}

// =====================================================================
// Setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (uint8_t pin : LDR_PINS) pinMode(pin, INPUT);

  if (!initPwm()) {
    Serial.println("ERROR: servo PWM initialization failed");
    while (true) delay(1000);
  }

  // CAUTION: the first pulse snaps each servo to HOME. Center manually before power-up.
  outputAxis(pan);
  delay(500);
  outputAxis(tilt);
  delay(500);

  sampleSensors(true);
  lastControl = lastSerial = millis();
  Serial.println("# Dual Axis Solar Tracker V3 | PID velocity + slew limit + night sleep");
  Serial.println("# DATA,ms,TL,TR,BL,BR,errPan,errTilt,panDeg,tiltDeg,panDegPerS,tiltDegPerS,state");
  Serial.println("# state: TRACK / HOLD / SENSOR_FAULT / HOMING / SLEEP");
  printHelp();
  printParams();
}

void loop() {
  pollSerial();

  uint32_t now = millis();
  if ((uint32_t)(now - lastControl) < CONTROL_MS) return;
  float dt = (uint32_t)(now - lastControl) / 1000.0f;
  lastControl = now;
  dt = constrain(dt, 0.001f, 0.10f);

  sampleSensors();
  float rawTotal = raw[0] + raw[1] + raw[2] + raw[3];
  float total = filtered[0] + filtered[1] + filtered[2] + filtered[3];
  float avgLight = total * 0.25f;
  bool fault = detectFault(now, rawTotal);

  // Night/day detection with hysteresis + confirmation time.
  bool dark = avgLight < tune.minLight;
  bool bright = avgLight >= tune.minLight * LIGHT_WAKE_RATIO;
  if (dark) { if (darkSince == 0) darkSince = now | 1; } else darkSince = 0;
  if (bright) { if (brightSince == 0) brightSince = now | 1; } else brightSince = 0;
  bool darkConfirmed = darkSince && (now - darkSince >= DARK_CONFIRM_MS);
  bool brightConfirmed = brightSince && (now - brightSince >= LIGHT_CONFIRM_MS);

  switch (mode) {
    case Mode::TRACKING: {
      if (darkConfirmed) {
        resetPid(pan);
        resetPid(tilt);
        mode = Mode::HOMING;
        stateName = "HOMING";
        break;
      }

      if (fault || total <= 0.0f) {
        // Freeze: decelerate smoothly to a stop and hold.
        resetPid(pan);
        resetPid(tilt);
        moveAxis(pan, 0.0f, dt);
        moveAxis(tilt, 0.0f, dt);
        stateName = fault ? "SENSOR_FAULT" : "HOLD";
        break;
      }

      float left   = filtered[0] + filtered[2];
      float right  = filtered[1] + filtered[3];
      float top    = filtered[0] + filtered[1];
      float bottom = filtered[2] + filtered[3];

      // Positive pan error: left brighter. Positive tilt error: top brighter.
      float vPan  = pidUpdate(pan,  (left - right) / total, 1.0f, dt);
      float vTilt = pidUpdate(tilt, (top - bottom) / total, tune.tiltRatio, dt);

      moveAxis(pan,  pan.direction  * vPan,  dt);
      moveAxis(tilt, tilt.direction * vTilt, dt);

      bool idle = !pan.moving && !tilt.moving &&
                  fabsf(pan.vel) < 0.05f && fabsf(tilt.vel) < 0.05f;
      stateName = idle ? "HOLD" : "TRACK";
      break;
    }

    case Mode::HOMING: {
      if (brightConfirmed) {
        mode = Mode::TRACKING;
        stateName = "TRACK";
        break;
      }
      bool panHome = homeAxis(pan, dt);
      bool tiltHome = homeAxis(tilt, dt);
      stateName = "HOMING";
      if (panHome && tiltHome) {
        outputAxis(pan);  // send the exact home pulse once before releasing
        outputAxis(tilt);
        mode = Mode::SLEEPING;
        stateName = "SLEEP";
      }
      break;
    }

    case Mode::SLEEPING: {
      if (brightConfirmed) {
        // Resume pulses at the stored home angle: no jump.
        pan.lastDuty = tilt.lastDuty = UINT32_MAX;
        resetPid(pan);
        resetPid(tilt);
        mode = Mode::TRACKING;
        stateName = "TRACK";
      } else {
        stateName = "SLEEP";
      }
      break;
    }
  }

  if (mode == Mode::SLEEPING) {
    releaseAxis(pan);
    releaseAxis(tilt);
  } else {
    outputAxis(pan);
    outputAxis(tilt);
  }

  if (csvEnabled && (uint32_t)(now - lastSerial) >= SERIAL_MS) {
    lastSerial = now;
    printCsv(now);
  }
}
