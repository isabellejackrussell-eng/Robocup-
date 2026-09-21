#include "perception/object_validation.h"

#include <math.h>

#include "config/robot_config.h"

namespace perception::validation {
namespace {
ValidationResult result;
}  // namespace

CandidateStatus validateWeightCandidate(const weight::WeightData& weightData,
                                        const wall::WallState&) {
  result = {};
  result.timestampMs = millis();
  if (!weightData.found) return result.status;

  result.weightDistanceMm = weightData.distanceMm;
  result.weightBearingDeg = weightData.bearingDeg;
  result.wallDistanceMm = wall::getWallDistanceAtBearing(weightData.bearingDeg);
  if (isnan(result.wallDistanceMm)) {
    result.status = CandidateStatus::UNCONFIRMED;
    return result.status;
  }

  result.separationMm = result.wallDistanceMm - result.weightDistanceMm;
  const bool nearCorner = wall::detectPossibleCorner();
  if (result.separationMm >= config::WALL_WEIGHT_SEPARATION_MARGIN_MM) {
    result.status = CandidateStatus::PLAUSIBLE_WEIGHT;
  } else if (nearCorner) {
    // Corner geometry is ambiguous; gather more frames instead of rejecting.
    result.status = CandidateStatus::UNCONFIRMED;
  } else {
    result.status = CandidateStatus::LIKELY_WALL_FEATURE;
  }
  return result.status;
}

bool confirmedWeightDetected() { return result.status == CandidateStatus::PLAUSIBLE_WEIGHT; }
float getValidatedWeightDistance() { return result.weightDistanceMm; }
float getValidatedWeightBearing() { return result.weightBearingDeg; }
const ValidationResult& getValidationResult() { return result; }

const __FlashStringHelper* statusName(CandidateStatus status) {
  switch (status) {
    case CandidateStatus::NONE: return F("NONE");
    case CandidateStatus::UNCONFIRMED: return F("UNCONFIRMED");
    case CandidateStatus::PLAUSIBLE_WEIGHT: return F("PLAUSIBLE_WEIGHT");
    case CandidateStatus::LIKELY_WALL_FEATURE: return F("LIKELY_WALL_FEATURE");
  }
  return F("UNKNOWN");
}

}  // namespace perception::validation

