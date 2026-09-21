#include "localisation/optical_flow.h"

#include <Bitcraze_PMW3901.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"

namespace localisation::optical_flow {
namespace {
Bitcraze_PMW3901 sensor(config::OPTICAL_FLOW_CS_PIN);
int16_t deltaX = 0;
int16_t deltaY = 0;
uint32_t lastReadMs = 0;
bool ready = false;
}  // namespace

bool initOpticalFlow() {
  ready = sensor.begin();
  debug::Log.println(ready ? F("[FLOW] PMW3901 ready") : F("[FLOW] ERROR: PMW3901 init failed"));
  return ready;
}

bool readOpticalFlow() {
  if (!ready) return false;
  sensor.readMotionCount(&deltaX, &deltaY);
  lastReadMs = millis();
  return true;
}

int16_t getDeltaXCounts() { return deltaX; }
int16_t getDeltaYCounts() { return deltaY; }

float getDeltaX() {
  if (config::OPTICAL_FLOW_COUNTS_PER_MM_X <= 0.0f) return NAN;
  return config::OPTICAL_FLOW_X_SIGN * deltaX / config::OPTICAL_FLOW_COUNTS_PER_MM_X;
}

float getDeltaY() {
  if (config::OPTICAL_FLOW_COUNTS_PER_MM_Y <= 0.0f) return NAN;
  return config::OPTICAL_FLOW_Y_SIGN * deltaY / config::OPTICAL_FLOW_COUNTS_PER_MM_Y;
}

bool calibrated() {
  return config::OPTICAL_FLOW_COUNTS_PER_MM_X > 0.0f &&
         config::OPTICAL_FLOW_COUNTS_PER_MM_Y > 0.0f;
}
bool healthy() { return ready && millis() - lastReadMs <= config::SENSOR_STALE_TIMEOUT_MS; }

}  // namespace localisation::optical_flow

