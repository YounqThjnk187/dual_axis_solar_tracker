/*
 * ================================================================
 * DUAL AXIS SOLAR TRACKER - ESP32
 * Improved firmware for 4 LDR + 2 positional SG90 servos
 * ================================================================
 *
 * Hardware (classic ESP32):
 *   LDR TL -> GPIO34
 *   LDR TR -> GPIO35
 *   LDR BL -> GPIO32
 *   LDR BR -> GPIO33
 *
 *   Pan servo  -> GPIO18
 *   Tilt servo -> GPIO19
 *
 * LDR divider assumed:
 *   3.3V -> LDR -> ADC node -> 10k resistor -> GND
 *   Brighter light should produce a HIGHER ADC reading.
 *
 * Servo power:
 *   Use a separate regulated 5V supply with sufficient current.
 *   Connect external supply GND to ESP32 GND.
 *
 * Features:
 *   - 12-bit ADC
 *   - Median-of-5 sampling per LDR
 *   - EMA low-pass filtering
 *   - Auto-calibration disabled (gain = 1.0 initially)
 *   - Normalized horizontal/vertical light error
 *   - Low-light / invalid-signal hold
 *   - P-only velocity controller for initial safe tuning
 *   - Servo speed and angle limits
 *   - Arduino ESP32 Core 2.x / 3.x LEDC compatibility
 *
 * IMPORTANT:
 *   1. Test sensor readings first with servos disconnected.
 *   2. Test one servo axis at a time.
 *   3. Verify PAN_DIRECTION and TILT_DIRECTION before unattended use.
 *   4. Adjust pulse limits for your actual servo/mechanics.
 * ================================================================
 */

#include <Arduino.h>
#include <math.h>

// ------------------------------------------------
// 1. PIN CONFIGURATION
// ------------------------------------------------
#define PIN_LDR_TL       34
#define PIN_LDR_TR       35
#define PIN_LDR_BL       32
#define PIN_LDR_BR       33

#define PIN_SERVO_PAN    18
#define PIN_SERVO_TILT   19

#define CH_SERVO_PAN     0
#define CH_SERVO_TILT    1

// ------------------------------------------------
// 2. PWM CONFIGURATION
// ------------------------------------------------
const uint32_t PWM_FREQ = 50;
const uint8_t PWM_RES = 13;
const float PWM_PERIOD_US = 1000000.0f / PWM_FREQ;
const float PWM_DUTY_FULL = (float)(1UL << PWM_RES);

// Arduino-ESP32 Core 3.x ledcWrite() uses pin;
// Core 2.x ledcWrite() uses channel.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define SERVO_PAN_ID   PIN_SERVO_PAN
  #define SERVO_TILT_ID  PIN_SERVO_TILT
#else
  #define SERVO_PAN_ID   CH_SERVO_PAN
  #define SERVO_TILT_ID  CH_SERVO_TILT
#endif

// ------------------------------------------------
// 3. TRACKER TUNING PARAMETERS
// ------------------------------------------------

// Median sampling
const uint8_t ADC_SAMPLE_COUNT = 5;
const uint16_t ADC_SAMPLE_DELAY_US = 150;

// EMA filter: larger = faster response, smaller = smoother
const float EMA_ALPHA = 0.20f;

// Servo angle limits
const float ANGLE_MIN = 10.0f;
const float ANGLE_MAX = 170.0f;

const float HOME_PAN = 90.0f;
const float HOME_TILT = 40.0f;

// Conservative initial pulse range for SG90.
// Check your own servo before widening this range.
const float SERVO_MIN_US = 1000.0f;
const float SERVO_MAX_US = 2000.0f;

// Normalized error deadband
const float DEADBAND = 0.05f;

// Minimum total raw ADC reading required for tracking.
// Increase this if the system moves in dim ambient light.
const float MIN_LIGHT_SUM = 500.0f;

// Conservative speed limit for initial testing
const float MAX_SPEED_DEG_S = 8.0f;

// P-only controller: output is commanded servo speed (deg/s)
const float KP_PAN = 25.0f;
const float KP_TILT = 25.0f;

// Direction depends on physical installation.
// If an axis moves AWAY from the light, change its sign.
const float PAN_DIRECTION = -1.0f;
const float TILT_DIRECTION = +1.0f;

// Main control loop
const uint32_t LOOP_INTERVAL_MS = 20;   // 50 Hz
const uint32_t SERIAL_INTERVAL_MS = 200;

// ------------------------------------------------
// 4. SENSOR STATE
// ------------------------------------------------
enum LdrIndex {
  LDR_TL = 0,
  LDR_TR,
  LDR_BL,
  LDR_BR,
  NUM_LDR
};

const uint8_t LDR_PINS[NUM_LDR] = {
  PIN_LDR_TL, PIN_LDR_TR, PIN_LDR_BL, PIN_LDR_BR
};

// Calibration intentionally disabled for initial testing.
// Set individual gains only after a controlled uniform-light calibration.
float ldrGain[NUM_LDR] = {1.0f, 1.0f, 1.0f, 1.0f};

float rawLdr[NUM_LDR] = {0.0f, 0.0f, 0.0f, 0.0f};
float ema[NUM_LDR] = {0.0f, 0.0f, 0.0f, 0.0f};

// ------------------------------------------------
// 5. SERVO STATE
// ------------------------------------------------
float currentPanAngle = HOME_PAN;
float currentTiltAngle = HOME_TILT;

unsigned long lastLoopTime = 0;
unsigned long lastSerialTime = 0;

// ------------------------------------------------
// 6. SENSOR READING: MEDIAN OF 5
// ------------------------------------------------
uint16_t readLdrMedian(uint8_t pin) {
  uint16_t samples[ADC_SAMPLE_COUNT];

  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
    samples[i] = analogRead(pin);
    delayMicroseconds(ADC_SAMPLE_DELAY_US);
  }

  // Insertion sort (small fixed array)
  for (uint8_t i = 1; i < ADC_SAMPLE_COUNT; i++) {
    uint16_t key = samples[i];
    int8_t j = (int8_t)i - 1;

    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  return samples[ADC_SAMPLE_COUNT / 2];
}

float readLdr(uint8_t index) {
  return (float)readLdrMedian(LDR_PINS[index]) * ldrGain[index];
}

void readAndFilterSensors() {
  for (uint8_t i = 0; i < NUM_LDR; i++) {
    rawLdr[i] = readLdr(i);

    ema[i] = EMA_ALPHA * rawLdr[i]
           + (1.0f - EMA_ALPHA) * ema[i];
  }
}

// ------------------------------------------------
// 7. SERVO PWM OUTPUT
// ------------------------------------------------
void servoWrite(uint8_t servoId, float angle) {
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);

  float pulseUs = SERVO_MIN_US
                + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180.0f;

  uint32_t duty = (uint32_t)(
    (pulseUs / PWM_PERIOD_US) * PWM_DUTY_FULL
  );

  ledcWrite(servoId, duty);
}

// ------------------------------------------------
// 8. P-ONLY VELOCITY CONTROLLER
// ------------------------------------------------
float computeP(float error, float kp) {
  if (fabsf(error) < DEADBAND) {
    return 0.0f;
  }

  return kp * error;
}

// ------------------------------------------------
// 9. PWM INITIALIZATION
// ------------------------------------------------
bool initializeServoPWM() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3

  bool panOK = ledcAttach(PIN_SERVO_PAN, PWM_FREQ, PWM_RES);
  bool tiltOK = ledcAttach(PIN_SERVO_TILT, PWM_FREQ, PWM_RES);

  return panOK && tiltOK;

#else

  ledcSetup(CH_SERVO_PAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_SERVO_PAN, CH_SERVO_PAN);

  ledcSetup(CH_SERVO_TILT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_SERVO_TILT, CH_SERVO_TILT);

  return true;

#endif
}

// ------------------------------------------------
// 10. SETUP
// ------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" ESP32 DUAL AXIS SOLAR TRACKER");
  Serial.println(" Improved firmware - 4 LDR + 2 SG90");
  Serial.println("========================================");

  // ADC configuration
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  for (uint8_t i = 0; i < NUM_LDR; i++) {
    pinMode(LDR_PINS[i], INPUT);
    ldrGain[i] = 1.0f;
  }

  // Initialize servo PWM
  if (!initializeServoPWM()) {
    Serial.println("ERROR: Failed to initialize servo PWM.");
    while (true) {
      delay(1000);
    }
  }

  // Send servos to home sequentially to reduce simultaneous inrush current
  currentPanAngle = HOME_PAN;
  currentTiltAngle = HOME_TILT;

  servoWrite(SERVO_PAN_ID, currentPanAngle);
  delay(500);

  servoWrite(SERVO_TILT_ID, currentTiltAngle);
  delay(500);

  Serial.println("LDR auto-calibration is DISABLED (gain = 1.0).");
  Serial.println("Keep sensors illuminated normally; starting EMA...");

  // Initialize EMA with actual sensor readings
  for (uint8_t i = 0; i < NUM_LDR; i++) {
    rawLdr[i] = readLdr(i);
    ema[i] = rawLdr[i];
  }

  lastLoopTime = millis();
  lastSerialTime = millis();

  Serial.println("System ready.");
  Serial.println("Monitoring format:");
  Serial.println("RAW[TL TR BL BR] EMA[TL TR BL BR] ErrH ErrV PAN TILT");
}

// ------------------------------------------------
// 11. MAIN CONTROL LOOP
// ------------------------------------------------
void loop() {
  unsigned long now = millis();

  if (now - lastLoopTime < LOOP_INTERVAL_MS) {
    return;
  }

  float dt = (now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;

  // Guard against an unexpectedly long loop interval
  if (dt <= 0.0f) {
    return;
  }
  if (dt > 0.1f) {
    dt = 0.1f;
  }

  // STEP 1: Read and filter all four LDRs
  readAndFilterSensors();

  // STEP 2: Calculate light sums
  const float left = ema[LDR_TL] + ema[LDR_BL];
  const float right = ema[LDR_TR] + ema[LDR_BR];

  const float top = ema[LDR_TL] + ema[LDR_TR];
  const float bottom = ema[LDR_BL] + ema[LDR_BR];

  const float filteredTotal = left + right;

  const float rawTotal =
      rawLdr[LDR_TL] + rawLdr[LDR_TR]
    + rawLdr[LDR_BL] + rawLdr[LDR_BR];

  float errorPan = 0.0f;
  float errorTilt = 0.0f;

  // STEP 3: Low-light / signal-loss protection.
  // If the raw readings are too low, hold the current servo positions.
  if (rawTotal < MIN_LIGHT_SUM ||
      filteredTotal < MIN_LIGHT_SUM) {

    servoWrite(SERVO_PAN_ID, currentPanAngle);
    servoWrite(SERVO_TILT_ID, currentTiltAngle);

    if (now - lastSerialTime >= SERIAL_INTERVAL_MS) {
      lastSerialTime = now;

      Serial.printf(
        "LOW LIGHT / INVALID SIGNAL | RawSum: %.0f | "
        "RAW[%.0f %.0f %.0f %.0f]\n",
        rawTotal,
        rawLdr[LDR_TL], rawLdr[LDR_TR],
        rawLdr[LDR_BL], rawLdr[LDR_BR]
      );
    }

    return;
  }

  // STEP 4: Normalized errors.
  // Positive ErrH means left side is brighter.
  // Positive ErrV means top side is brighter.
  errorPan = (left - right) / filteredTotal;
  errorTilt = (top - bottom) / filteredTotal;

  // STEP 5: P-only controller -> requested speed (deg/s)
  float panSpeed = computeP(errorPan, KP_PAN);
  float tiltSpeed = computeP(errorTilt, KP_TILT);

  panSpeed = constrain(
    panSpeed,
    -MAX_SPEED_DEG_S,
    MAX_SPEED_DEG_S
  );

  tiltSpeed = constrain(
    tiltSpeed,
    -MAX_SPEED_DEG_S,
    MAX_SPEED_DEG_S
  );

  // STEP 6: Update commanded servo positions.
  // Direction signs must be verified with the real mechanical assembly.
  currentPanAngle = constrain(
    currentPanAngle + PAN_DIRECTION * panSpeed * dt,
    ANGLE_MIN,
    ANGLE_MAX
  );

  currentTiltAngle = constrain(
    currentTiltAngle + TILT_DIRECTION * tiltSpeed * dt,
    ANGLE_MIN,
    ANGLE_MAX
  );

  // STEP 7: Output PWM
  servoWrite(SERVO_PAN_ID, currentPanAngle);
  servoWrite(SERVO_TILT_ID, currentTiltAngle);

  // STEP 8: Serial diagnostics
  if (now - lastSerialTime >= SERIAL_INTERVAL_MS) {
    lastSerialTime = now;

    Serial.printf(
      "RAW[TL:%4.0f TR:%4.0f BL:%4.0f BR:%4.0f] "
      "EMA[TL:%4.0f TR:%4.0f BL:%4.0f BR:%4.0f] "
      "ErrH:%+.3f ErrV:%+.3f "
      "PAN:%.1f TILT:%.1f\n",

      rawLdr[LDR_TL],
      rawLdr[LDR_TR],
      rawLdr[LDR_BL],
      rawLdr[LDR_BR],

      ema[LDR_TL],
      ema[LDR_TR],
      ema[LDR_BL],
      ema[LDR_BR],

      errorPan,
      errorTilt,

      currentPanAngle,
      currentTiltAngle
    );
  }
}
