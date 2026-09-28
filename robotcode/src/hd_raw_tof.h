#ifndef HD_RAW_TOF_H
#define HD_RAW_TOF_H

#include <Arduino.h>

namespace hd_raw_tof {

constexpr uint8_t kSensorCount = 6;
constexpr uint8_t kExpanderI2cAddress = 0x71;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint32_t kMeasurementTimingBudgetUs = 50000;
constexpr uint32_t kMeasurementPeriodMs = 50;

enum class SensorModel : uint8_t {
  vl53l0x,
  vl53l1x,
};

struct Reading {
  uint16_t distanceMm;
  bool valid;
  bool timedOut;
};

using Readings = Reading[kSensorCount];

// Initialises the SX1509 expander and all six range sensors.
// Returns false if the expander or any sensor fails to initialise.
bool initialise();

// Reads every sensor. A timeout invalidates only that sample; it does not
// permanently disable the sensor on future reads.
void readAll(Readings& readings);

bool isSensorInitialised(uint8_t sensorIndex);
SensorModel sensorModel(uint8_t sensorIndex);
const char* sensorModelName(uint8_t sensorIndex);

}  // namespace hd_raw_tof

#endif
