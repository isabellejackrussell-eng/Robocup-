#pragma once

#include "localisation/pose_estimator.h"
#include "perception/sensor_state.h"

namespace navigation {

struct MotionCommand {
  float leftSpeed = 0.0f;
  float rightSpeed = 0.0f;
  bool stop = true;
};

MotionCommand navigateTowardWeight(const perception::SensorState& sensors);
MotionCommand avoidObstacle(const perception::SensorState& sensors);
MotionCommand navigateHome(const localisation::pose::Pose& pose);
MotionCommand searchForWeight();
MotionCommand calculateMotionCommand();
const MotionCommand& getMotionCommand();

}  // namespace navigation

