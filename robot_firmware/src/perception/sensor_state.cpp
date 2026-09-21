#include "perception/sensor_state.h"

#include "config/robot_config.h"
#include "perception/capture_detection.h"
#include "perception/object_validation.h"
#include "perception/wall_detection.h"
#include "perception/weight_detection.h"

namespace perception {
namespace {
SensorState state;
}  // namespace

void updateSensorState() {
  const weight::WeightData& weightData = weight::getWeightData();
  const wall::WallState& wallState = wall::getWallState();
  state.weightCandidate = weightData.found;
  state.confirmedWeight = validation::confirmedWeightDetected();
  state.weightDistanceMm = weightData.found ? weightData.distanceMm : NAN;
  state.weightBearingDeg = weightData.found ? weightData.bearingDeg : NAN;
  state.wallLeftDistanceMm = wallState.wallLeftDistanceMm;
  state.wallFrontDistanceMm = wallState.wallFrontDistanceMm;
  state.wallRightDistanceMm = wallState.wallRightDistanceMm;
  state.wallLeftValid = wallState.leftValid;
  state.wallFrontValid = wallState.frontValid;
  state.wallRightValid = wallState.rightValid;
  state.possibleLeftCorner = wallState.possibleLeftCorner;
  state.possibleRightCorner = wallState.possibleRightCorner;
  state.weightInCaptureZone = capture::weightInCaptureZone();
  // Healthy means the essential 8x8 stream is current. Optional/unconfigured
  // sensors advertise their own validity and do not masquerade as zero.
  state.sensorsHealthy = weightData.timestampMs != 0 &&
                         millis() - weightData.timestampMs <= 2 * config::SENSOR_STALE_TIMEOUT_MS;
  state.timestampMs = millis();
}

const SensorState& getSensorState() { return state; }

}  // namespace perception
