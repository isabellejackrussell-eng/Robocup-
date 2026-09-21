#include "sensors/tof8x8.h"

#include <DFRobot_MatrixLidar.h>

#include "debug/debug_port.h"

namespace sensors::tof8x8 {
namespace {
DFRobot_MatrixLidar_I2C sensor(config::TOF8X8_I2C_ADDRESS);
Frame latest;
Frame calibration;
bool initialised = false;
}  // namespace

bool init8x8() {
  if (sensor.begin() != 0) {
    debug::Log.println(F("[8X8] ERROR: sensor begin failed"));
    return false;
  }
  if (sensor.setRangingMode(eMatrix_8X8) != 0) {
    debug::Log.println(F("[8X8] ERROR: could not select 8x8 mode"));
    return false;
  }
  initialised = true;
  debug::Log.println(F("[8X8] Ready; send 'c' over USB/Bluetooth to calibrate empty scene"));
  return true;
}

bool read8x8() {
  if (!initialised) return false;
  sensor.getAllData(latest.distanceMm);
  latest.valid = true;
  latest.timestampMs = millis();
  return true;
}

const Frame& get8x8Frame() { return latest; }

bool calibrate8x8() {
  if (!latest.valid && !read8x8()) return false;
  calibration = latest;
  debug::Log.println(F("[8X8] Empty-scene calibration frame captured"));
  return true;
}

bool hasCalibrationFrame() { return calibration.valid; }
const Frame& getCalibrationFrame() { return calibration; }
bool healthy() { return initialised && latest.valid; }

}  // namespace sensors::tof8x8

