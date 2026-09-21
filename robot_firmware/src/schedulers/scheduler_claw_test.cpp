#include "schedulers/scheduler_claw_test.h"

#include <Arduino.h>

#include "claw/claw.h"
#include "config/robot_config.h"
#include "debug/debug_port.h"

namespace schedulers::claw_test {
namespace {
enum class Phase : uint8_t { RELEASING, RELEASE_PAUSE, GRABBING, GRAB_PAUSE };
Phase phase = Phase::RELEASING;
uint32_t pauseStartedMs = 0;
}  // namespace

void setup() {
  claw::initClaw();
  debug::Log.println(F("[CLAW TEST] RELEASE / HOME"));
}

void loop() {
  claw::update();
  switch (phase) {
    case Phase::RELEASING:
      if (claw::clawFinished()) {
        debug::Log.println(F("[CLAW TEST] RELEASE COMPLETE"));
        pauseStartedMs = millis();
        phase = Phase::RELEASE_PAUSE;
      }
      break;
    case Phase::RELEASE_PAUSE:
      if (millis() - pauseStartedMs >= config::CLAW_TEST_PAUSE_MS) {
        debug::Log.println(F("[CLAW TEST] GRAB"));
        claw::grab();
        phase = Phase::GRABBING;
      }
      break;
    case Phase::GRABBING:
      if (claw::clawFinished()) {
        debug::Log.println(F("[CLAW TEST] GRAB COMPLETE"));
        pauseStartedMs = millis();
        phase = Phase::GRAB_PAUSE;
      }
      break;
    case Phase::GRAB_PAUSE:
      if (millis() - pauseStartedMs >= config::CLAW_TEST_PAUSE_MS) {
        debug::Log.println(F("[CLAW TEST] RELEASE"));
        claw::release();
        phase = Phase::RELEASING;
      }
      break;
  }
}

}  // namespace schedulers::claw_test

