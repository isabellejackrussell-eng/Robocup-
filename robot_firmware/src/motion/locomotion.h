#pragma once

namespace motion {

void initLocomotion();
void setTrackSpeeds(int leftSpeed, int rightSpeed);
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotion();
int getCommandedLeftSpeed();
int getCommandedRightSpeed();

}  // namespace motion

