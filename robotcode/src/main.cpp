#include <Arduino.h>

#include "filter_weightDetect.h"
#include "hd_raw_tof8x8.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 3000;
constexpr uint32_t kRetryPeriodMs = 1000;
constexpr uint32_t kSamplePeriodMs = 100;

void printFrame(const hd_raw_tof8x8::Frame& frame) {
  for (uint8_t row = 0; row < hd_raw_tof8x8::kGridSize; ++row) {
    Serial.print('Y');
    Serial.print(row);
    Serial.print(": ");
    for (uint8_t column = 0;
         column < hd_raw_tof8x8::kGridSize;
         ++column) {
      Serial.print(frame[row * hd_raw_tof8x8::kGridSize + column]);
      Serial.print(',');
    }
    Serial.println();
  }
}

bool readFrame(hd_raw_tof8x8::Frame& frame) {
  if (hd_raw_tof8x8::readFrame(frame)) {
    return true;
  }

  Serial.println("8x8 frame read failed");
  return false;
}

void calibrateWhenRequested() {
  if (Serial.available() <= 0) {
    return;
  }

  const char command = static_cast<char>(Serial.read());
  if (command != 'c' && command != 'C') {
    return;
  }

  hd_raw_tof8x8::Frame calibrationFrame;
  if (!readFrame(calibrationFrame)) {
    Serial.println("Background calibration failed");
    return;
  }

  filter_weightDetect::calibrateBackground(calibrationFrame);
  Serial.println("Background calibrated");
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  const uint32_t serialWaitStartedMs = millis();
  while (!Serial && millis() - serialWaitStartedMs < kSerialWaitMs) {
  }

  pinMode(kIoPowerPin, OUTPUT);
  digitalWrite(kIoPowerPin, HIGH);
  delay(500);

  while (!hd_raw_tof8x8::initialise()) {
    Serial.println("begin error, retrying...");
    delay(kRetryPeriodMs);
  }

  filter_weightDetect::clearBackgroundCalibration();
  Serial.println("begin success");
  Serial.println("init success, starting readings...");
  Serial.println(
      "Point sensor at empty scene, then send 'c' to calibrate background.");
}

void loop() {
  calibrateWhenRequested();

  hd_raw_tof8x8::Frame frame;
  if (!readFrame(frame)) {
    delay(kSamplePeriodMs);
    return;
  }

  printFrame(frame);
  Serial.println("------------------------------");

  const filter_weightDetect::WeightResult result =
      filter_weightDetect::detectWeight(frame, true);
  filter_weightDetect::printWeightResult(result);
  Serial.println("------------------------------");

  delay(kSamplePeriodMs);
}
