#include "schedulers/scheduler_full_integration_test.h"

#include <Arduino.h>

#include "claw/claw.h"
#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "debug/robot_debug.h"
#include "motion/locomotion.h"
#include "schedulers/scheduler_common.h"
#include "state/robot_state.h"

namespace schedulers::full_integration_test {
namespace {
uint32_t lastSensorMs = 0;
uint32_t lastPerceptionMs = 0;
uint32_t lastLocalisationMs = 0;
uint32_t lastNavigationMs = 0;
uint32_t lastDebugMs = 0;
bool grabIssued = false;
}  // namespace

void setup() {
  motion::initLocomotion();
  claw::initClaw();
  state::initRobotState();
  common::initSensorSuite();
  common::initLocalisationSuite();
  debug::printHelp();
  debug::Log.println(F("[FULL] IDLE for safety. Calibrate with 'c', then send 'g' to start"));
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
    state::updateRobotState();
  }
  if (now - lastLocalisationMs >= config::LOCALISATION_UPDATE_PERIOD_MS) {
    lastLocalisationMs = now;
    common::readLocalisation();
    common::updateLocalisation();
  }
  if (now - lastNavigationMs >= config::NAV_UPDATE_PERIOD_MS) {
    lastNavigationMs = now;
    common::applyNavigationCommand();
  }

  if (state::getRobotMode() == state::RobotMode::CAPTURING) {
    motion::stopMotion();
    if (!grabIssued) {
      debug::Log.println(F("[FULL] Capture zone confirmed -> STOP -> GRAB"));
      claw::grab();
      grabIssued = true;
    }
  } else {
    grabIssued = false;
  }

  if (now - lastDebugMs >= config::DEBUG_PRINT_PERIOD_MS) {
    lastDebugMs = now;
    common::printAllDebug(false);
  }
}

}  // namespace schedulers::full_integration_test
