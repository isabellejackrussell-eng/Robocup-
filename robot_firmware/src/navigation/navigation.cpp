#include "navigation/navigation.h"

#include <Arduino.h>
#include <math.h>

#include "config/robot_config.h"
#include "localisation/home_vector.h"
#include "state/robot_state.h"

namespace navigation {
namespace {
MotionCommand latest;

MotionCommand steer(float forwardSpeed, float bearingDeg) {
  const float correction = constrain(bearingDeg * config::BEARING_STEERING_GAIN,
                                     -forwardSpeed, forwardSpeed);
  MotionCommand command;
  command.leftSpeed = constrain(forwardSpeed + correction,
                                -static_cast<float>(config::MOTOR_MAX_POWER),
                                static_cast<float>(config::MOTOR_MAX_POWER));
  command.rightSpeed = constrain(forwardSpeed - correction,
                                 -static_cast<float>(config::MOTOR_MAX_POWER),
                                 static_cast<float>(config::MOTOR_MAX_POWER));
  command.stop = false;
  return command;
}
}  // namespace

MotionCommand navigateTowardWeight(const perception::SensorState& sensors) {
  if (!sensors.confirmedWeight || isnan(sensors.weightDistanceMm) ||
      sensors.weightDistanceMm <= config::APPROACH_STOP_DISTANCE_MM) {
    return {};
  }
  return steer(config::APPROACH_SPEED, sensors.weightBearingDeg);
}

MotionCommand avoidObstacle(const perception::SensorState& sensors) {
  MotionCommand command;
  command.stop = false;
  const bool leftHasMoreRoom = !sensors.wallLeftValid ||
      (sensors.wallRightValid && sensors.wallLeftDistanceMm > sensors.wallRightDistanceMm);
  if (leftHasMoreRoom) {
    command.leftSpeed = -config::AVOID_SPEED;
    command.rightSpeed = config::AVOID_SPEED;
  } else {
    command.leftSpeed = config::AVOID_SPEED;
    command.rightSpeed = -config::AVOID_SPEED;
  }
  return command;
}

MotionCommand navigateHome(const localisation::pose::Pose& poseValue) {
  const localisation::home::HomeVector vector = localisation::home::calculateHomeVector(poseValue);
  if (!poseValue.translationCalibrated || vector.distanceMm <= config::HOME_ARRIVAL_DISTANCE_MM) {
    return {};
  }
  return steer(config::HOME_SPEED, vector.bearingDeg);
}

MotionCommand searchForWeight() {
  MotionCommand command;
  command.leftSpeed = -config::SEARCH_SPEED;
  command.rightSpeed = config::SEARCH_SPEED;
  command.stop = false;
  return command;
}

MotionCommand calculateMotionCommand() {
  const perception::SensorState& sensors = perception::getSensorState();
  switch (state::getRobotMode()) {
    case state::RobotMode::SEARCHING:
      latest = searchForWeight();
      break;
    case state::RobotMode::APPROACHING_WEIGHT:
      latest = navigateTowardWeight(sensors);
      break;
    case state::RobotMode::AVOIDING_OBSTACLE:
      latest = avoidObstacle(sensors);
      break;
    case state::RobotMode::RETURNING_HOME:
      latest = navigateHome(localisation::pose::getPose());
      break;
    default:
      latest = {};
      break;
  }
  return latest;
}

const MotionCommand& getMotionCommand() { return latest; }

}  // namespace navigation

