#ifndef TOF_DIAGNOSTIC_H
#define TOF_DIAGNOSTIC_H

#include <Arduino.h>

namespace tof_diagnostic {

constexpr uint8_t kSensorCount = 8;
constexpr uint8_t kPreferredExpanderAddress = 0x71;
constexpr uint8_t kDefaultTofAddress = 0x29;
constexpr uint32_t kMeasurementPeriodMs = 50;
constexpr uint32_t kTimingBudgetUs = 50000;

// Runs the bus, expander, XSHUT, sensor-ID, and address-assignment tests.
// Returns the number of sensors which were successfully initialised.
uint8_t begin();

// Prints one line of live readings. Call repeatedly from loop().
void printReadings();

}  // namespace tof_diagnostic

#endif
