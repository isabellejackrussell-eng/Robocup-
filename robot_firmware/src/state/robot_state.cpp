#include "state/robot_state.h"

#include <math.h>

#include "claw/claw.h"
#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "perception/sensor_state.h"

namespace state {
namespace {
RobotMode mode = RobotMode::IDLE;
bool returnHomeRequested = false;

void transition(RobotMode next) {
  if (next == mode) return;
  debug::Log.print(F("[FSM] "));
  debug::Log.print(robotModeName(mode));
  debug::Log.print(F(" -> "));
  debug::Log.println(robotModeName(next));
  mode = next;
}
}  // namespace

void initRobotState() {
  mode = RobotMode::IDLE;
  returnHomeRequested = false;
}

void startAutonomous() { transition(RobotMode::SEARCHING); }
void stopAutonomous() {
  returnHomeRequested = false;
  transition(RobotMode::IDLE);
}
void requestReturnHome() { returnHomeRequested = true; }
void setError() { transition(RobotMode::ERROR); }

void updateRobotState() {
  if (mode == RobotMode::IDLE || mode == RobotMode::ERROR || mode == RobotMode::DEPOSITING) return;
  const perception::SensorState& sensors = perception::getSensorState();
  if (returnHomeRequested) {
    transition(RobotMode::RETURNING_HOME);
    return;
  }
  if (mode == RobotMode::CAPTURING) {
    if (claw::clawFinished() && !sensors.weightInCaptureZone) {
      transition(RobotMode::SEARCHING);
    }
    return;
  }
  if (sensors.weightInCaptureZone) {
    transition(RobotMode::CAPTURING);
    return;
  }
  if (sensors.wallFrontValid &&
      sensors.wallFrontDistanceMm <= config::OBSTACLE_STOP_DISTANCE_MM) {
    transition(RobotMode::AVOIDING_OBSTACLE);
  } else if (sensors.confirmedWeight) {
    transition(RobotMode::APPROACHING_WEIGHT);
  } else {
    transition(RobotMode::SEARCHING);
  }
}

RobotMode getRobotMode() { return mode; }

const __FlashStringHelper* robotModeName(RobotMode value) {
  switch (value) {
    case RobotMode::IDLE: return F("IDLE");
    case RobotMode::SEARCHING: return F("SEARCHING");
    case RobotMode::APPROACHING_WEIGHT: return F("APPROACHING_WEIGHT");
    case RobotMode::CAPTURING: return F("CAPTURING");
    case RobotMode::AVOIDING_OBSTACLE: return F("AVOIDING_OBSTACLE");
    case RobotMode::RETURNING_HOME: return F("RETURNING_HOME");
    case RobotMode::DEPOSITING: return F("DEPOSITING");
    case RobotMode::ERROR: return F("ERROR");
  }
  return F("UNKNOWN");
}

}  // namespace state
