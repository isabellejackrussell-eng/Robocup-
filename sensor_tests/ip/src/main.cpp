#include <Arduino.h>

namespace {

constexpr uint8_t kInductiveSensorPin = A6;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kSerialWaitMs = 3000;
constexpr uint32_t kReportIntervalMs = 250;

int previousRawState = -1;
uint32_t lastReportAtMs = 0;

void printReading(int rawState, bool changed) {
  Serial.print(changed ? "[CHANGE] " : "[READ]   ");
  Serial.print("pin=");
  Serial.print(kInductiveSensorPin);
  Serial.print(" raw=");
  Serial.print(rawState);
  Serial.print(rawState == HIGH ? " (HIGH)" : " (LOW)");

  // Show both interpretations so this test does not assume the sensor's
  // electrical polarity before it has been measured.
  Serial.print("  active-high metal=");
  Serial.print(rawState == HIGH ? "YES" : "NO");
  Serial.print("  active-low metal=");
  Serial.println(rawState == LOW ? "YES" : "NO");
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);

  const uint32_t serialWaitStartedAt = millis();
  while (!Serial && millis() - serialWaitStartedAt < kSerialWaitMs) {
    delay(10);
  }

  // Use a plain input so the test observes the signal supplied by the sensor
  // board without imposing an internal pull-up or pull-down.
  pinMode(kInductiveSensorPin, INPUT);

  Serial.println();
  Serial.println("Inductive proximity sensor raw-input test");
  Serial.println("========================================");
  Serial.print("Reading Teensy A6, digital pin ");
  Serial.println(kInductiveSensorPin);
  Serial.println("Move metal toward and away from the sensor.");
  Serial.println("The raw state should change when the sensor LED changes.");
  Serial.println();

  previousRawState = digitalRead(kInductiveSensorPin);
  printReading(previousRawState, true);
  lastReportAtMs = millis();
}

void loop() {
  const uint32_t now = millis();
  const int rawState = digitalRead(kInductiveSensorPin);

  if (rawState != previousRawState) {
    previousRawState = rawState;
    printReading(rawState, true);
    lastReportAtMs = now;
    return;
  }

  if (now - lastReportAtMs >= kReportIntervalMs) {
    printReading(rawState, false);
    lastReportAtMs = now;
  }
}
