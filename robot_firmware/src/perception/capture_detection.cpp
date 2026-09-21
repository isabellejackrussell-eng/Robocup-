#include "perception/capture_detection.h"

#include <math.h>

#include "config/robot_config.h"

namespace perception::capture {
namespace {
CaptureState state;

float filterValue(float previous, float current) {
  if (isnan(previous)) return current;
  return previous + config::WALL_FILTER_ALPHA * (current - previous);
}
}  // namespace

void filterCaptureReadings(const sensors::capture::CaptureSensorRaw& raw) {
  const uint32_t now = millis();
  const bool leftValid = raw.left.isFresh(now, config::SENSOR_STALE_TIMEOUT_MS);
  const bool rightValid = raw.right.isFresh(now, config::SENSOR_STALE_TIMEOUT_MS);
  if (leftValid) state.filteredLeftMm = filterValue(state.filteredLeftMm, raw.left.distanceMm);
  if (rightValid) state.filteredRightMm = filterValue(state.filteredRightMm, raw.right.distanceMm);
  state.leftTriggered = leftValid && state.filteredLeftMm <= config::CAPTURE_DISTANCE_THRESHOLD_MM;
  state.rightTriggered = rightValid && state.filteredRightMm <= config::CAPTURE_DISTANCE_THRESHOLD_MM;
  state.weightPresent = config::CAPTURE_REQUIRES_BOTH_SENSORS
                            ? (state.leftTriggered && state.rightTriggered)
                            : (state.leftTriggered || state.rightTriggered);
  state.confidence = (state.leftTriggered ? 0.5f : 0.0f) +
                     (state.rightTriggered ? 0.5f : 0.0f);
  state.timestampMs = now;
}

bool weightInCaptureZone() { return state.weightPresent; }
float captureConfidence() { return state.confidence; }
const CaptureState& getCaptureState() { return state; }

}  // namespace perception::capture

