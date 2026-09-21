#include "schedulers/scheduler_sensor_claw_test.h"

#include <Arduino.h>

#include "claw/claw.h"
#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "debug/robot_debug.h"
#include "perception/capture_detection.h"
#include "schedulers/scheduler_common.h"

namespace schedulers::sensor_claw_test {
namespace {
enum class CapturePhase : uint8_t { WAITING_FOR_WEIGHT, GRABBING, GRAB_COMPLETE, RELEASING };
CapturePhase phase = CapturePhase::WAITING_FOR_WEIGHT;
uint32_t lastSensorMs = 0;
uint32_t lastPerceptionMs = 0;
uint32_t lastDebugMs = 0;
}  // namespace

void setup() {
  claw::initClaw();
  common::initSensorSuite();
  debug::Log.println(F("[SENSOR+CLAW] WAITING_FOR_WEIGHT; both capture sensors must trigger"));
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

  const bool captured = perception::capture::weightInCaptureZone();
  switch (phase) {
    case CapturePhase::WAITING_FOR_WEIGHT:
      if (captured) {
        debug::Log.println(F("[SENSOR+CLAW] WEIGHT_DETECTED -> GRABBING"));
        claw::grab();
        phase = CapturePhase::GRABBING;
      }
      break;
    case CapturePhase::GRABBING:
      if (claw::clawFinished()) {
        debug::Log.println(F("[SENSOR+CLAW] GRAB_COMPLETE (latched)"));
        phase = CapturePhase::GRAB_COMPLETE;
      }
      break;
    case CapturePhase::GRAB_COMPLETE:
      if (!captured) {
        debug::Log.println(F("[SENSOR+CLAW] Capture zone clear -> RELEASE"));
        claw::release();
        phase = CapturePhase::RELEASING;
      }
      break;
    case CapturePhase::RELEASING:
      if (claw::clawFinished()) {
        debug::Log.println(F("[SENSOR+CLAW] WAITING_FOR_WEIGHT"));
        phase = CapturePhase::WAITING_FOR_WEIGHT;
      }
      break;
  }

  if (now - lastDebugMs >= config::DEBUG_PRINT_PERIOD_MS) {
    lastDebugMs = now;
    debug::printSensorDebug();
  }
}

}  // namespace schedulers::sensor_claw_test
