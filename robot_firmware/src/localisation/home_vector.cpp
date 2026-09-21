#include "localisation/home_vector.h"

#include <Arduino.h>
#include <math.h>

namespace localisation::home {
namespace {
HomeVector current;

float normalise(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}
}  // namespace

HomeVector calculateHomeVector(const pose::Pose& poseValue) {
  current.xMm = -poseValue.xMm;
  current.yMm = -poseValue.yMm;
  current.distanceMm = sqrtf(current.xMm * current.xMm + current.yMm * current.yMm);
  const float worldBearing = atan2f(current.yMm, current.xMm) * RAD_TO_DEG;
  current.bearingDeg = normalise(worldBearing - poseValue.headingDeg);
  return current;
}

float getHomeDistance() { return current.distanceMm; }
float getHomeBearing() { return current.bearingDeg; }
const HomeVector& getHomeVector() { return current; }

}  // namespace localisation::home
