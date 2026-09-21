#pragma once

#include "localisation/pose_estimator.h"

namespace localisation::home {

struct HomeVector {
  float xMm = 0.0f;
  float yMm = 0.0f;
  float distanceMm = 0.0f;
  float bearingDeg = 0.0f;
};

HomeVector calculateHomeVector(const pose::Pose& pose);
float getHomeDistance();
float getHomeBearing();
const HomeVector& getHomeVector();

}  // namespace localisation::home

