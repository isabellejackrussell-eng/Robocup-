#include "hd_raw_xy.h"

#include <Bitcraze_PMW3901.h>

namespace hd_raw_xy {
namespace {

Bitcraze_PMW3901 opticalFlow(kChipSelectPin);
bool initialised = false;

}  // namespace

bool initialise() {
  initialised = opticalFlow.begin();
  return initialised;
}

bool readMotion(MotionReading& reading) {
  reading = {0, 0, millis(), false};

  if (!initialised) {
    return false;
  }

  opticalFlow.readMotionCount(
      &reading.deltaXCounts,
      &reading.deltaYCounts);
  reading.timestampMs = millis();
  reading.valid = true;
  return true;
}

bool isInitialised() {
  return initialised;
}

}  // namespace hd_raw_xy
