#include "sensors/capture_sensors.h"

#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "sensors/range_tof.h"

namespace sensors::capture {
namespace {
CaptureSensorRaw raw;
}  // namespace

bool initCaptureSensors() {
  const bool configured = range_tof::configured(config::CAPTURE_LEFT_TOF) &&
                          range_tof::configured(config::CAPTURE_RIGHT_TOF);
  if (!configured) {
    debug::Log.println(F("[CAPTURE] NOT CONFIGURED: both capture ToF XSHUT pins required"));
  }
  return configured;
}

void readCaptureLeft() { raw.left = range_tof::get(config::CAPTURE_LEFT_TOF); }
void readCaptureRight() { raw.right = range_tof::get(config::CAPTURE_RIGHT_TOF); }

void readCaptureSensors() {
  readCaptureLeft();
  readCaptureRight();
  raw.timestampMs = millis();
}

const CaptureSensorRaw& getCaptureSensorRaw() { return raw; }

}  // namespace sensors::capture
