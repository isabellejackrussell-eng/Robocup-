#include "schedulers/scheduler_sensor_test.h"

#include <Arduino.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "debug/robot_debug.h"
#include "schedulers/scheduler_common.h"

namespace schedulers::sensor_test {
namespace {
uint32_t lastSensorMs = 0;
uint32_t lastPerceptionMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastGridMs = 0;
}  // namespace

void setup() {
  common::initSensorSuite();
  debug::printHelp();
  debug::Log.println(F("[SENSOR TEST] Safe mode: motors and claw are not initialised"));
}

void loop() {
  common::serviceFastTasks();
  const uint32_t now = millis();
  if (now - lastSensorMs >= config::SENSOR_READ_PERIOD_MS) {
    lastSensorMs = now;
    common::readSensorSuite();
  }
  if (now - lastPerceptionMs >= config::PERCEPTION_UPDATE_PERIOD_MS) {
    lastPerceptionMs = now;
    common::processPerception();
  }
  if (now - lastDebugMs >= config::DEBUG_PRINT_PERIOD_MS) {
    lastDebugMs = now;
    debug::printSensorDebug();
  }
  if (now - lastGridMs >= 2000) {
    lastGridMs = now;
    debug::print8x8Grid();
  }
}

}  // namespace schedulers::sensor_test

