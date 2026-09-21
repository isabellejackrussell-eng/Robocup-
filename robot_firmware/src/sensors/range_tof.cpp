#include "sensors/range_tof.h"

#include <Wire.h>
#include <SparkFunSX1509.h>
#include <VL53L1X.h>

#include "debug/debug_port.h"

namespace sensors::range_tof {
namespace {
SX1509 io;
VL53L1X sensors[config::RANGE_TOF_COUNT];
DistanceReading readings[config::RANGE_TOF_COUNT];
bool ready[config::RANGE_TOF_COUNT] = {};
bool expanderReady = false;
}  // namespace

bool configured(uint8_t index) {
  return index < config::RANGE_TOF_COUNT &&
         config::RANGE_TOF_XSHUT_PINS[index] != config::PIN_UNASSIGNED;
}

bool initRangeToFSensors() {
  bool anyConfigured = false;
  for (uint8_t i = 0; i < config::RANGE_TOF_COUNT; ++i) anyConfigured |= configured(i);
  if (!anyConfigured) {
    debug::Log.println(F("[RANGE TOF] NOT CONFIGURED: set XSHUT pins in robot_config.h"));
    return false;
  }

  Wire.begin();
  Wire.setClock(400000);
  expanderReady = io.begin(config::SX1509_I2C_ADDRESS);
  if (!expanderReady) {
    debug::Log.println(F("[RANGE TOF] ERROR: SX1509 not found"));
    return false;
  }

  for (uint8_t i = 0; i < config::RANGE_TOF_COUNT; ++i) {
    if (!configured(i)) continue;
    io.pinMode(config::RANGE_TOF_XSHUT_PINS[i], OUTPUT);
    io.digitalWrite(config::RANGE_TOF_XSHUT_PINS[i], LOW);
  }

  for (uint8_t i = 0; i < config::RANGE_TOF_COUNT; ++i) {
    if (!configured(i)) continue;
    io.pinMode(config::RANGE_TOF_XSHUT_PINS[i], INPUT);
    delay(10);
    sensors[i].setTimeout(100);
    if (!sensors[i].init()) {
      debug::Log.print(F("[RANGE TOF] ERROR initialising index "));
      debug::Log.println(i);
      continue;
    }
    sensors[i].setAddress(config::RANGE_TOF_I2C_ADDRESSES[i]);
    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(config::RANGE_TOF_TIMING_BUDGET_US);
    sensors[i].startContinuous(config::RANGE_TOF_PERIOD_MS);
    ready[i] = true;
    debug::Log.print(F("[RANGE TOF] Ready index "));
    debug::Log.println(i);
  }
  return true;
}

bool read(uint8_t index) {
  if (index >= config::RANGE_TOF_COUNT || !ready[index]) return false;
  const uint16_t distance = sensors[index].read();
  const bool valid = !sensors[index].timeoutOccurred() && distance > 0 && distance < 4000;
  readings[index].distanceMm = valid ? static_cast<float>(distance) : NAN;
  readings[index].valid = valid;
  readings[index].timestampMs = millis();
  return valid;
}

void readAll() {
  for (uint8_t i = 0; i < config::RANGE_TOF_COUNT; ++i) read(i);
}

DistanceReading get(uint8_t index) {
  if (index >= config::RANGE_TOF_COUNT) return {};
  return readings[index];
}

bool healthy(uint8_t index) {
  return index < config::RANGE_TOF_COUNT && ready[index] &&
         readings[index].isFresh(millis(), config::SENSOR_STALE_TIMEOUT_MS);
}

}  // namespace sensors::range_tof
