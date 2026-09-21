#pragma once

#include <Arduino.h>

namespace state {

enum class RobotMode : uint8_t {
  IDLE,
  SEARCHING,
  APPROACHING_WEIGHT,
  CAPTURING,
  AVOIDING_OBSTACLE,
  RETURNING_HOME,
  DEPOSITING,
  ERROR
};

void initRobotState();
void startAutonomous();
void stopAutonomous();
void requestReturnHome();
void setError();
void updateRobotState();
RobotMode getRobotMode();
const __FlashStringHelper* robotModeName(RobotMode mode);

}  // namespace state
