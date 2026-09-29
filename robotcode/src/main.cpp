#include <Arduino.h>

#include "filter_weightDetect.h"
#include "hd_raw_tof8x8.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 3000;
constexpr size_t kCalibrationFrameCount = 10;
constexpr uint8_t kSamplesPerStage = 20;
constexpr uint32_t kSamplePeriodMs = 100;

struct TestStage {
  const char* name;
  int16_t nominalDistanceMm;
};

constexpr TestStage kWeightStages[] = {
    {"weight_front", 0},
    {"weight_10mm", 10},
    {"weight_100mm", 100},
    {"weight_150mm", 150},
    {"weight_200mm", 200},
};

size_t nextWeightStage = 0;
bool testComplete = false;

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

void printDataHeader() {
  Serial.print(
      "record,stage,nominal_distance_mm,sample,timestamp_ms,"
      "filter_found,filter_column,filter_row,filter_distance_mm,"
      "filter_cluster_size");
  for (uint8_t row = 0; row < hd_raw_tof8x8::kGridSize; ++row) {
    for (uint8_t column = 0;
         column < hd_raw_tof8x8::kGridSize;
         ++column) {
      Serial.print(",p");
      Serial.print(row);
      Serial.print(column);
    }
  }
  Serial.println();
}

void printDataRow(
    const TestStage& stage,
    uint8_t sample,
    const hd_raw_tof8x8::Frame& frame,
    const filter_weightDetect::WeightResult& result) {
  Serial.print("WEIGHT_DATA,");
  Serial.print(stage.name);
  Serial.print(',');
  Serial.print(stage.nominalDistanceMm);
  Serial.print(',');
  Serial.print(sample);
  Serial.print(',');
  Serial.print(millis());
  Serial.print(',');
  Serial.print(result.found ? 1 : 0);
  Serial.print(',');
  Serial.print(result.averageColumn, 3);
  Serial.print(',');
  Serial.print(result.averageRow, 3);
  Serial.print(',');
  Serial.print(result.averageDistanceMm);
  Serial.print(',');
  Serial.print(result.clusterSize);

  for (size_t point = 0; point < hd_raw_tof8x8::kPointCount; ++point) {
    Serial.print(',');
    Serial.print(frame[point]);
  }
  Serial.println();
}

void captureStage(const TestStage& stage) {
  Serial.println();
  Serial.println("========================================");
  Serial.print("[CAPTURE START] ");
  Serial.print(stage.name);
  Serial.print(" nominal_distance_mm=");
  Serial.println(stage.nominalDistanceMm);

  for (uint8_t sample = 1; sample <= kSamplesPerStage; ++sample) {
    const uint32_t sampleStartedMs = millis();
    hd_raw_tof8x8::Frame frame;
    if (!hd_raw_tof8x8::readFrame(frame)) {
      Serial.print("[READ FAILED] sample=");
      Serial.println(sample);
    } else {
      Serial.print("[8x8 SAMPLE] stage=");
      Serial.print(stage.name);
      Serial.print(" sample=");
      Serial.print(sample);
      Serial.print('/');
      Serial.println(kSamplesPerStage);
      printFrame(frame);

      Serial.print("[8x8 FILTER] ");
      const filter_weightDetect::WeightResult result =
          filter_weightDetect::detectWeight(frame, true);
      filter_weightDetect::printWeightResult(result);
      printDataRow(stage, sample, frame, result);
      Serial.println("------------------------------");
    }

    const uint32_t elapsedMs = millis() - sampleStartedMs;
    if (elapsedMs < kSamplePeriodMs) {
      delay(kSamplePeriodMs - elapsedMs);
    }
  }

  Serial.print("[CAPTURE END] ");
  Serial.println(stage.name);
  Serial.println("========================================");
}

bool calibrateEmptyScene() {
  Serial.println();
  Serial.println("[CALIBRATION] Capturing 10 empty-scene frames...");

  hd_raw_tof8x8::Frame frames[kCalibrationFrameCount];
  for (size_t frame = 0; frame < kCalibrationFrameCount; ++frame) {
    if (!hd_raw_tof8x8::readFrame(frames[frame])) {
      Serial.print("[CALIBRATION] Frame read failed at ");
      Serial.println(frame + 1);
      return false;
    }
    delay(50);
  }

  if (!filter_weightDetect::calibrateBackgroundAveraged(
          frames, kCalibrationFrameCount)) {
    return false;
  }

  Serial.println("[CALIBRATION] Background calibrated");
  captureStage({"no_weight", -1});
  return true;
}

void promptForNextStage() {
  if (nextWeightStage >=
      sizeof(kWeightStages) / sizeof(kWeightStages[0])) {
    testComplete = true;
    Serial.println();
    Serial.println("[TEST COMPLETE] All weight positions captured");
    Serial.println("The sensor will now remain idle.");
    return;
  }

  const TestStage& stage = kWeightStages[nextWeightStage];
  Serial.println();
  Serial.print("Place the weight at ");
  if (stage.nominalDistanceMm == 0) {
    Serial.print("the front of the 8x8 sensor (0 mm test position)");
  } else {
    Serial.print(stage.nominalDistanceMm);
    Serial.print(" mm from the front of the 8x8 sensor");
  }
  Serial.println(".");
  Serial.println("Keep it centred, then send 'n' to capture this stage.");
}

char readCommand() {
  char command = '\0';
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming != '\r' && incoming != '\n') {
      command = incoming;
    }
  }
  return command;
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  const uint32_t serialWaitStartedMs = millis();
  while (!Serial && millis() - serialWaitStartedMs < kSerialWaitMs) {
    delay(10);
  }

  pinMode(kIoPowerPin, OUTPUT);
  digitalWrite(kIoPowerPin, HIGH);
  delay(500);

  Serial.println();
  Serial.println("8x8 weight-detection distance test");
  Serial.println("========================================");

  while (!hd_raw_tof8x8::initialise()) {
    Serial.println("[INIT] 8x8 sensor failed; retrying in one second...");
    delay(1000);
  }
  Serial.println("[INIT] 8x8 sensor OK");

  filter_weightDetect::clearBackgroundCalibration();
  printDataHeader();
  Serial.println();
  Serial.println("Remove all weights from the sensor view.");
  Serial.println("Send 'c' to calibrate and capture the no-weight baseline.");
}

void loop() {
  if (testComplete || Serial.available() <= 0) {
    return;
  }

  const char command = readCommand();
  if (!filter_weightDetect::isBackgroundCalibrated()) {
    if (command != 'c' && command != 'C') {
      Serial.println("Send 'c' while the 8x8 view is empty to calibrate.");
      return;
    }

    if (!calibrateEmptyScene()) {
      Serial.println("[CALIBRATION] Failed; clear the view and send 'c' again.");
      return;
    }

    promptForNextStage();
    return;
  }

  if (command != 'n' && command != 'N') {
    Serial.println("Send 'n' when the weight is positioned for capture.");
    return;
  }

  captureStage(kWeightStages[nextWeightStage]);
  ++nextWeightStage;
  promptForNextStage();
}
