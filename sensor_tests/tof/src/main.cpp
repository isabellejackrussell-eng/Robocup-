#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3F;

// Just testing the one sensor on XSHUT0 for now.
const uint8_t sensorCount = 1;
const uint8_t xshutPins[sensorCount] = {0}; // SX1509 pin 0 == XSHUT0

SX1509 io;
VL53L1X sensors[sensorCount];

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C

  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("Failed to find SX1509 expander!");
    while (1) delay(1000);
  }

  // Hold sensor in reset by driving XSHUT low.
  for (uint8_t i = 0; i < sensorCount; i++) {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }

  // Bring sensor out of reset, then init.
  for (uint8_t i = 0; i < sensorCount; i++) {
    // Release XSHUT to input so it floats high (don't drive it high directly,
    // it's not level-shifted on this board).
    io.pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1) delay(1000);
    }

    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(50000);
    sensors[i].startContinuous(50);
  }

  Serial.println("VL53L1X initialized. Starting readings...");
}

void loop() {
  for (uint8_t i = 0; i < sensorCount; i++) {
    Serial.print(sensors[i].read());
    if (sensors[i].timeoutOccurred()) Serial.print(" TIMEOUT");
    Serial.print('\t');
  }
  Serial.println();
}