/*
  DUAL AXIS SOLAR TRACKER V2 - ESP32 classic + 4 LDR + 2 SG90
  Requires Arduino-ESP32 Core 2.x or 3.x (no ESP32Servo dependency).

  LDR wiring assumed: 3.3V -- LDR -- ADC pin -- 10k resistor -- GND
  More light -> higher ADC. Change wiring/logic if yours differs.

  PAN GPIO18, TILT GPIO19
  TL GPIO34, TR GPIO35, BL GPIO32, BR GPIO33
  Servo supply: external regulated 5V, common GND with ESP32.

  CONTROL: normalized optical error -> P velocity -> integrate to servo angle.
  This is NOT a conventional PID controller.
*/
#include <Arduino.h>
#include <math.h>
#include <esp_arduino_version.h>

constexpr uint8_t LDR_PINS[4] = {34, 35, 32, 33}; // TL TR BL BR
constexpr uint8_t PAN_PIN = 18, TILT_PIN = 19;
constexpr uint8_t PAN_CHANNEL = 0, TILT_CHANNEL = 1;
constexpr uint32_t PWM_FREQ = 50;
constexpr uint8_t PWM_BITS = 13;
constexpr float PWM_PERIOD_US = 20000.0f;
constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_BITS) - 1;

// ---- Adjustable settings ----
constexpr float PAN_HOME = 90.0f, TILT_HOME = 90.0f;
constexpr float PAN_MIN = 20.0f, PAN_MAX = 160.0f;
constexpr float TILT_MIN = 20.0f, TILT_MAX = 160.0f;
constexpr float PULSE_MIN_US = 1000.0f, PULSE_MAX_US = 2000.0f;

// Verify these signs physically. If the servo moves AWAY from the light,
// invert the corresponding sign (+1 <-> -1).
constexpr float PAN_DIRECTION = -1.0f;
constexpr float TILT_DIRECTION = +1.0f;

constexpr float EMA_ALPHA = 0.20f;
constexpr float DEADBAND = 0.06f;
constexpr float KP_PAN = 20.0f;   // deg/s per normalized error
constexpr float KP_TILT = 15.0f;  // deg/s per normalized error
constexpr float MAX_SPEED_DEG_S = 6.0f;
constexpr float LIGHT_OFF = 500.0f;  // sum of 4 ADC values
constexpr float LIGHT_ON = 750.0f;
constexpr uint32_t CONTROL_MS = 40;
constexpr uint32_t SERIAL_MS = 200;
constexpr uint8_t MEDIAN_N = 5;
constexpr uint16_t ADC_SAMPLE_GAP_US = 150;

// Optional suspicious single-channel saturation detection.
// Only triggers when one LDR is at full scale while the other three
// together are relatively dark for >2 seconds; not a universal fault test.
constexpr bool ENABLE_SATURATION_GUARD = true;
constexpr uint16_t SATURATED_ADC = 4090;
constexpr float OTHER_THREE_MAX_SUM = 350.0f;
constexpr uint32_t FAULT_CONFIRM_MS = 2000;

float raw[4] = {0, 0, 0, 0};
float filtered[4] = {0, 0, 0, 0};
float gain[4] = {1, 1, 1, 1}; // calibrate only with controlled uniform light
float panAngle = PAN_HOME, tiltAngle = TILT_HOME;
float errPan = 0, errTilt = 0;
float panSpeed = 0, tiltSpeed = 0;
bool tracking = false;
bool sensorFault = false;
uint32_t suspiciousSince[4] = {0, 0, 0, 0};
uint32_t lastControl = 0, lastSerial = 0;

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
    filtered[i] = initialize ? raw[i] :
      (EMA_ALPHA * raw[i] + (1.0f - EMA_ALPHA) * filtered[i]);
  }
}

bool initPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  bool panOK = ledcAttach(PAN_PIN, PWM_FREQ, PWM_BITS);
  bool tiltOK = ledcAttach(TILT_PIN, PWM_FREQ, PWM_BITS);
  return panOK && tiltOK;
#else
  double f1 = ledcSetup(PAN_CHANNEL, PWM_FREQ, PWM_BITS);
  double f2 = ledcSetup(TILT_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PAN_PIN, PAN_CHANNEL);
  ledcAttachPin(TILT_PIN, TILT_CHANNEL);
  return f1 > 0 && f2 > 0;
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

void outputServos() {
  writeServo(PAN_PIN, PAN_CHANNEL, panAngle, PAN_MIN, PAN_MAX);
  writeServo(TILT_PIN, TILT_CHANNEL, tiltAngle, TILT_MIN, TILT_MAX);
}

float continuousDeadbandP(float error, float kp) {
  float amount = fabsf(error) - DEADBAND;
  if (amount <= 0.0f) return 0.0f;
  return copysignf(kp * amount, error);
}

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

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (uint8_t pin : LDR_PINS) pinMode(pin, INPUT);

  if (!initPwm()) {
    Serial.println("ERROR: servo PWM initialization failed");
    while (true) delay(1000);
  }

  // CAUTION: initial servo commands can move loaded mechanisms abruptly.
  // Confirm mechanical clearance and manually center before power-up.
  writeServo(PAN_PIN, PAN_CHANNEL, panAngle, PAN_MIN, PAN_MAX);
  delay(500);
  writeServo(TILT_PIN, TILT_CHANNEL, tiltAngle, TILT_MIN, TILT_MAX);
  delay(500);

  sampleSensors(true);
  lastControl = lastSerial = millis();
  Serial.println("# Dual Axis Solar Tracker V2 | P velocity controller");
  Serial.println("# DATA,ms,TL,TR,BL,BR,errPan,errTilt,panDeg,tiltDeg,panDegPerS,tiltDegPerS,state");
  Serial.println("# state: TRACK / DEADBAND / LOW_LIGHT / SENSOR_FAULT");
}

void loop() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastControl) < CONTROL_MS) return;
  float dt = (uint32_t)(now - lastControl) / 1000.0f;
  lastControl = now;
  dt = constrain(dt, 0.001f, 0.10f);

  sampleSensors();
  float rawTotal = raw[0] + raw[1] + raw[2] + raw[3];
  float total = filtered[0] + filtered[1] + filtered[2] + filtered[3];
  sensorFault = detectFault(now, rawTotal);

  // Hysteresis: once stopped, both sums must exceed LIGHT_ON to resume.
  if (tracking) {
    if (rawTotal < LIGHT_OFF || total < LIGHT_OFF) tracking = false;
  } else {
    if (rawTotal >= LIGHT_ON && total >= LIGHT_ON) tracking = true;
  }

  errPan = 0.0f;
  errTilt = 0.0f;
  panSpeed = 0.0f;
  tiltSpeed = 0.0f;
  const char *state = "LOW_LIGHT";

  if (sensorFault) {
    state = "SENSOR_FAULT";
  } else if (tracking && total > 0.0f) {
    float left = filtered[0] + filtered[2];
    float right = filtered[1] + filtered[3];
    float top = filtered[0] + filtered[1];
    float bottom = filtered[2] + filtered[3];

    // Positive pan error: left brighter. Positive tilt error: top brighter.
    errPan = (left - right) / total;
    errTilt = (top - bottom) / total;

    panSpeed = constrain(continuousDeadbandP(errPan, KP_PAN),
                         -MAX_SPEED_DEG_S, MAX_SPEED_DEG_S);
    tiltSpeed = constrain(continuousDeadbandP(errTilt, KP_TILT),
                          -MAX_SPEED_DEG_S, MAX_SPEED_DEG_S);

    panAngle = constrain(panAngle + PAN_DIRECTION * panSpeed * dt,
                         PAN_MIN, PAN_MAX);
    tiltAngle = constrain(tiltAngle + TILT_DIRECTION * tiltSpeed * dt,
                          TILT_MIN, TILT_MAX);

    state = (panSpeed == 0.0f && tiltSpeed == 0.0f) ? "DEADBAND" : "TRACK";
  }

  // Servo commands continue holding their last requested positions.
  outputServos();

  if ((uint32_t)(now - lastSerial) >= SERIAL_MS) {
    lastSerial = now;
    printCsv(now, state);
  }
}
