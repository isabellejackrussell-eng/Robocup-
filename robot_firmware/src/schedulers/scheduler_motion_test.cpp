#include "schedulers/scheduler_motion_test.h"

#include <Arduino.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "motion/locomotion.h"

namespace schedulers::motion_test {
namespace {
enum class Phase : uint8_t { FORWARD, STOP_1, REVERSE, STOP_2, LEFT, STOP_3, RIGHT, STOP_4 };
Phase phase = Phase::FORWARD;
uint32_t phaseStartedMs = 0;

uint32_t duration(Phase value) {
  switch (value) {
    case Phase::FORWARD:
    case Phase::REVERSE:
    case Phase::LEFT:
    case Phase::RIGHT:
      return config::MOTION_TEST_ACTION_MS;
    default:
      return config::MOTION_TEST_STOP_MS;
  }
}

void enter(Phase value) {
  phase = value;
  phaseStartedMs = millis();
  switch (phase) {
    case Phase::FORWARD:
      debug::Log.println(F("[MOTION TEST] FORWARD"));
      motion::moveForward(config::MOTOR_TEST_SPEED);
      break;
    case Phase::REVERSE:
      debug::Log.println(F("[MOTION TEST] REVERSE"));
      motion::moveBackward(config::MOTOR_TEST_SPEED);
      break;
    case Phase::LEFT:
      debug::Log.println(F("[MOTION TEST] TURN LEFT"));
      motion::turnLeft(config::TURN_TEST_SPEED);
      break;
    case Phase::RIGHT:
      debug::Log.println(F("[MOTION TEST] TURN RIGHT"));
      motion::turnRight(config::TURN_TEST_SPEED);
      break;
    default:
      debug::Log.println(F("[MOTION TEST] STOP"));
      motion::stopMotion();
      break;
  }
}
}  // namespace

void setup() {
  motion::initLocomotion();
  enter(Phase::FORWARD);
}

void loop() {
  if (millis() - phaseStartedMs < duration(phase)) return;
  switch (phase) {
    case Phase::FORWARD: enter(Phase::STOP_1); break;
    case Phase::STOP_1: enter(Phase::REVERSE); break;
    case Phase::REVERSE: enter(Phase::STOP_2); break;
    case Phase::STOP_2: enter(Phase::LEFT); break;
    case Phase::LEFT: enter(Phase::STOP_3); break;
    case Phase::STOP_3: enter(Phase::RIGHT); break;
    case Phase::RIGHT: enter(Phase::STOP_4); break;
    case Phase::STOP_4: enter(Phase::FORWARD); break;
  }
}

}  // namespace schedulers::motion_test

