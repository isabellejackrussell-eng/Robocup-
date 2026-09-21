#include "localisation/pose_estimator.h"

#include <Arduino.h>
#include <math.h>

#include "localisation/encoders.h"
#include "localisation/imu.h"
#include "localisation/optical_flow.h"

namespace localisation::pose {
namespace {
Pose current;
float previousLeftDistance = 0.0f;
float previousRightDistance = 0.0f;
bool haveEncoderBaseline = false;
}  // namespace

void initialisePose() {
  current = {};
  previousLeftDistance = 0.0f;
  previousRightDistance = 0.0f;
  haveEncoderBaseline = false;
}

void updatePose() {
  if (imu::healthy()) {
    current.headingDeg = imu::getHeading();
    current.headingValid = true;
  }

  float robotRightMm = 0.0f;
  float robotForwardMm = 0.0f;
  bool haveTranslation = false;
  if (optical_flow::calibrated() && optical_flow::healthy()) {
    robotRightMm = optical_flow::getDeltaX();
    robotForwardMm = optical_flow::getDeltaY();
    haveTranslation = !isnan(robotRightMm) && !isnan(robotForwardMm);
  } else if (encoders::calibrated()) {
    const float left = encoders::getLeftDistance();
    const float right = encoders::getRightDistance();
    if (haveEncoderBaseline) {
      robotForwardMm = ((left - previousLeftDistance) + (right - previousRightDistance)) * 0.5f;
      haveTranslation = true;
    }
    previousLeftDistance = left;
    previousRightDistance = right;
    haveEncoderBaseline = true;
  }

  current.translationCalibrated = optical_flow::calibrated() || encoders::calibrated();
  if (!haveTranslation) return;
  const float headingRad = current.headingDeg * DEG_TO_RAD;
  current.xMm += robotForwardMm * cosf(headingRad) + robotRightMm * sinf(headingRad);
  current.yMm += robotForwardMm * sinf(headingRad) - robotRightMm * cosf(headingRad);
}

const Pose& getPose() { return current; }
float getX() { return current.xMm; }
float getY() { return current.yMm; }
float getHeading() { return current.headingDeg; }

}  // namespace localisation::pose

