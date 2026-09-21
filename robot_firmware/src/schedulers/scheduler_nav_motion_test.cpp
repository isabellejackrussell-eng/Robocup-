#include "schedulers/scheduler_nav_motion_test.h"

#include <Arduino.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "debug/robot_debug.h"
#include "motion/locomotion.h"
#include "schedulers/scheduler_common.h"
#include "state/robot_state.h"

namespace schedulers::nav_motion_test {
namespace {
uint32_t lastLocalisationMs = 0;
uint32_t lastNavigationMs = 0;
uint32_t lastDebugMs = 0;
}  // namespace

void setup() {
  motion::initLocomotion();
  state::initRobotState();
  common::initLocalisationSuite();
  debug::printHelp();
  debug::Log.println(F("[NAV+MOTION] IDLE. Send 'g' to search/turn, 'r' to test return-home, 'x' to stop"));
}

void loop() {
  common::pollDebugCommands();
  const uint32_t now = millis();
  if (now - lastLocalisationMs >= config::LOCALISATION_UPDATE_PERIOD_MS) {
    lastLocalisationMs = now;
    common::readLocalisation();
    common::updateLocalisation();
  }
  if (now - lastNavigationMs >= config::NAV_UPDATE_PERIOD_MS) {
    lastNavigationMs = now;
    common::applyNavigationCommand();
  }
  if (now - lastDebugMs >= config::DEBUG_PRINT_PERIOD_MS) {
    lastDebugMs = now;
    debug::printPoseDebug();
    debug::printNavigationDebug();
  }
}

}  // namespace schedulers::nav_motion_test

