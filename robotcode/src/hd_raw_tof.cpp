#include "hd_raw_tof.h"

#include <Wire.h>
#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

namespace hd_raw_tof {
namespace {

constexpr uint8_t kXshutPins[kSensorCount] = {
    0, 1, 2, 3,
    // 4, 5,  // Set 2 is not currently fitted.
};
constexpr uint8_t kSensorI2cAddresses[kSensorCount] = {
    0x30, 0x31, 0x32, 0x34,
    // 0x35, 0x36,  // Set 2 is not currently fitted.
};
constexpr SensorModel kSensorModels[kSensorCount] = {
    SensorModel::vl53l1x,
    SensorModel::vl53l1x,
    SensorModel::vl53l1x,
    SensorModel::vl53l0x,
    // SensorModel::vl53l0x,  // Set 2 is not currently fitted.
    // SensorModel::vl53l0x,
};

SX1509 ioExpander;
VL53L1X vl53l1xSensors[kSensorCount];
VL53L0X vl53l0xSensors[kSensorCount];
bool sensorInitialised[kSensorCount] = {};

bool initialiseVl53l0x(uint8_t sensorIndex) {
  VL53L0X& sensor = vl53l0xSensors[sensorIndex];
  sensor.setTimeout(500);

  if (!sensor.init()) {
    return false;
  }

  sensor.setAddress(kSensorI2cAddresses[sensorIndex]);
  sensor.startContinuous(kMeasurementPeriodMs);
  return true;
}

bool initialiseVl53l1x(uint8_t sensorIndex) {
  VL53L1X& sensor = vl53l1xSensors[sensorIndex];
  sensor.setTimeout(500);

  if (!sensor.init()) {
    return false;
  }

  sensor.setAddress(kSensorI2cAddresses[sensorIndex]);
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(kMeasurementTimingBudgetUs);
  sensor.startContinuous(kMeasurementPeriodMs);
  return true;
}

}  // namespace

bool initialise() {
  Wire.begin();
  Wire.setClock(kI2cClockHz);

  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    sensorInitialised[sensorIndex] = false;
  }

  if (!ioExpander.begin(kExpanderI2cAddress)) {
    return false;
  }

  // All sensors start at the same address, so hold all of them in reset.
  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    ioExpander.pinMode(kXshutPins[sensorIndex], OUTPUT);
    ioExpander.digitalWrite(kXshutPins[sensorIndex], LOW);
  }

  bool allInitialised = true;

  // Release and re-address the sensors one at a time.
  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    ioExpander.pinMode(kXshutPins[sensorIndex], INPUT);
    delay(10);

    if (kSensorModels[sensorIndex] == SensorModel::vl53l0x) {
      sensorInitialised[sensorIndex] = initialiseVl53l0x(sensorIndex);
    } else {
      sensorInitialised[sensorIndex] = initialiseVl53l1x(sensorIndex);
    }

    if (!sensorInitialised[sensorIndex]) {
      allInitialised = false;
      ioExpander.pinMode(kXshutPins[sensorIndex], OUTPUT);
      ioExpander.digitalWrite(kXshutPins[sensorIndex], LOW);
    }
  }

  return allInitialised;
}

void readAll(Readings& readings) {
  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    Reading& reading = readings[sensorIndex];
    reading = {0, false, false};

    if (!sensorInitialised[sensorIndex]) {
      continue;
    }

    if (kSensorModels[sensorIndex] == SensorModel::vl53l0x) {
      VL53L0X& sensor = vl53l0xSensors[sensorIndex];
      reading.distanceMm = sensor.readRangeContinuousMillimeters();
      reading.timedOut = sensor.timeoutOccurred();
    } else {
      VL53L1X& sensor = vl53l1xSensors[sensorIndex];
      reading.distanceMm = sensor.read();
      reading.timedOut = sensor.timeoutOccurred();
    }

    reading.valid = !reading.timedOut && reading.distanceMm > 0;
  }
}

bool isSensorInitialised(uint8_t sensorIndex) {
  return sensorIndex < kSensorCount && sensorInitialised[sensorIndex];
}

SensorModel sensorModel(uint8_t sensorIndex) {
  if (sensorIndex >= kSensorCount) {
    return SensorModel::vl53l1x;
  }
  return kSensorModels[sensorIndex];
}

const char* sensorModelName(uint8_t sensorIndex) {
  return sensorModel(sensorIndex) == SensorModel::vl53l0x
      ? "VL53L0X"
      : "VL53L1X";
}

}  // namespace hd_raw_tof
