#ifndef HD_MOVE_SERVO_ARM_H
#define HD_MOVE_SERVO_ARM_H

#include <Arduino.h>

namespace hd_move_servoArm {

constexpr uint8_t kServoPin = 25;
constexpr uint8_t kInitialAngleDegrees = 90;
constexpr uint8_t kMinimumAngleDegrees = 0;
constexpr uint8_t kMaximumAngleDegrees = 200;

bool initialise(uint8_t initialAngleDegrees = kInitialAngleDegrees);
bool setAngle(int angleDegrees);
bool isInitialised();
uint8_t getCommandedAngle();

}  // namespace hd_move_servoArm

#endif
