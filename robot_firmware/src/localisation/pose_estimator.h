#pragma once

namespace localisation::pose {

struct Pose {
  float xMm = 0.0f;
  float yMm = 0.0f;
  float headingDeg = 0.0f;
  bool translationCalibrated = false;
  bool headingValid = false;
};

void initialisePose();
void updatePose();
const Pose& getPose();
float getX();
float getY();
float getHeading();

}  // namespace localisation::pose

