#pragma once

#include <Arduino.h>

namespace localisation::optical_flow {

bool initOpticalFlow();
bool readOpticalFlow();
int16_t getDeltaXCounts();
int16_t getDeltaYCounts();
float getDeltaX();
float getDeltaY();
bool calibrated();
bool healthy();

}  // namespace localisation::optical_flow

