#pragma once

#include <Arduino.h>

#include "hd_raw_encoder.h"
#include "hd_raw_imu.h"
#include "hd_raw_xy.h"

namespace filter_positionData {

// One collection of the three raw localisation inputs. No position estimate or
// sensor fusion is performed yet.
struct PositionData {
  hd_raw_imu::Reading imu;
  hd_raw_xy::MotionReading opticalFlow;
  hd_raw_encoder::Reading encoders;

  uint32_t collectedAtMs;
  bool imuValid;
  bool opticalFlowValid;
  bool encodersValid;
};

// Reads each HD module once. The return value is true only when all three
// sources are valid, while each individual validity flag remains available for
// partial-data operation.
bool collect(PositionData& data);

bool allSourcesValid(const PositionData& data);

}  // namespace filter_positionData
