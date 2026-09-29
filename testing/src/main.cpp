#include <Arduino.h>

#include "tof.h"

namespace {
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kPrintIntervalMs = 250;
uint32_t lastPrintMs = 0;
}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  while (!Serial && millis() < 3000) {
  }

  tof_diagnostic::begin();
  lastPrintMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPrintMs >= kPrintIntervalMs) {
    lastPrintMs = now;
    tof_diagnostic::printReadings();
  }
}
