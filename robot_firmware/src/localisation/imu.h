#pragma once

namespace localisation::imu {

bool initIMU();
bool calibrateIMU();
bool readIMU();
float getHeading();
float getAngularVelocity();
bool healthy();

}  // namespace localisation::imu

