#pragma once

#include "perception/wall_detection.h"
#include "perception/weight_detection.h"

namespace perception::validation {

enum class CandidateStatus : uint8_t {
  NONE,
  UNCONFIRMED,
  PLAUSIBLE_WEIGHT,
  LIKELY_WALL_FEATURE
};

struct ValidationResult {
  CandidateStatus status = CandidateStatus::NONE;
  float weightDistanceMm = NAN;
  float weightBearingDeg = NAN;
  float wallDistanceMm = NAN;
  float separationMm = NAN;
  uint32_t timestampMs = 0;
};

CandidateStatus validateWeightCandidate(const weight::WeightData& weightData,
                                        const wall::WallState& wallState);
bool confirmedWeightDetected();
float getValidatedWeightDistance();
float getValidatedWeightBearing();
const ValidationResult& getValidationResult();
const __FlashStringHelper* statusName(CandidateStatus status);

}  // namespace perception::validation

