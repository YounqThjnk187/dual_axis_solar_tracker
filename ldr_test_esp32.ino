/*
 * LDR CHARACTERIZATION TEST - ESP32
 * 4 LDRs, raw ADC logging over Serial
 *
 * Wiring assumed:
 *   TL -> GPIO34
 *   TR -> GPIO35
 *   BL -> GPIO32
 *   BR -> GPIO33
 *
 * Divider assumed:
 *   3.3V -> LDR -> ADC node -> 10k resistor -> GND
 * With this wiring, brighter light should generally produce a higher ADC value.
 *
 * Serial output format:
 *   DATA,millis,TL,TR,BL,BR
 *
 * No servo control and no automatic calibration in this test.
 */

#include <Arduino.h>

#define PIN_LDR_TL 34
#define PIN_LDR_TR 35
#define PIN_LDR_BL 32
#define PIN_LDR_BR 33

const uint8_t LDR_PINS[4] = {
  PIN_LDR_TL, PIN_LDR_TR, PIN_LDR_BL, PIN_LDR_BR
};

const unsigned long SAMPLE_INTERVAL_MS = 50; // 20 samples/second

unsigned long lastSampleMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12); // ESP32 ADC nominal range: 0..4095

  // ESP32 Arduino Core 2.x / 3.x compatibility:
  // Use the default ADC attenuation unless your board/core requires otherwise.
  // For classic ESP32, 11 dB is commonly used for a wider input range.
#if defined(ADC_11db)
  analogSetAttenuation(ADC_11db);
#endif

  // Discard a few initial conversions after startup.
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 4; j++) {
      analogRead(LDR_PINS[j]);
    }
    delay(10);
  }

  Serial.println("LDR_TEST_READY");
  Serial.println("FORMAT: DATA,millis,TL,TR,BL,BR");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;

    const int tl = analogRead(PIN_LDR_TL);
    const int tr = analogRead(PIN_LDR_TR);
    const int bl = analogRead(PIN_LDR_BL);
    const int br = analogRead(PIN_LDR_BR);

    Serial.printf(
      "DATA,%lu,%d,%d,%d,%d\n",
      now, tl, tr, bl, br
    );
  }
}
