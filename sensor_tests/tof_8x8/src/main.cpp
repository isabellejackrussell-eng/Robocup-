#include <Arduino.h>
#include "DFRobot_MatrixLidar.h"
#include "weight_detect.h"

DFRobot_MatrixLidar_I2C tof(0x33); // Default I2C address
uint16_t buf[64];                  // 8x8 = 64 distance points, in mm

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  while (tof.begin() != 0) {
    Serial.println("begin error, retrying...");
    delay(1000);
  }
  Serial.println("begin success");

  while (tof.setRangingMode(eMatrix_8X8) != 0) {   // capital X
    Serial.println("failed to set 8x8 mode, retrying...");
    delay(1000);
  }
  Serial.println("init success, starting readings...");
}

void loop() {
  tof.getAllData(buf);

  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("Y");
    Serial.print(i);
    Serial.print(": ");
    for (uint8_t j = 0; j < 8; j++) {
      Serial.print(buf[i * 8 + j]);
      Serial.print(",");
    }
    Serial.println();
  }
  Serial.println("------------------------------");

  // NEW: run detection on the same buf array
  WeightResult result = detectWeight(buf);
  printResult(result);
  Serial.println("------------------------------");

  delay(100);
}