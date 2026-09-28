#include "filter_positionData.h"

namespace filter_positionData {

bool collect(PositionData& data) {
  data = {};

  data.imuValid = hd_raw_imu::read(data.imu);
  data.opticalFlowValid = hd_raw_xy::readMotion(data.opticalFlow);
  data.encodersValid = hd_raw_encoder::read(data.encoders);
  data.collectedAtMs = millis();

  return allSourcesValid(data);
}

bool allSourcesValid(const PositionData& data) {
  return data.imuValid && data.opticalFlowValid && data.encodersValid;
}

}  // namespace filter_positionData
