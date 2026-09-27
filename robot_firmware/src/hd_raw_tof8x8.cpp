#include "hd_raw_tof8x8.h"

#include <DFRobot_MatrixLidar.h>

namespace hd_raw_tof8x8 {
namespace {

DFRobot_MatrixLidar_I2C sensor(kDefaultI2cAddress);
bool initialised = false;

}  // namespace

bool initialise() {
  initialised = false;

  if (sensor.begin() != 0) {
    return false;
  }

  if (sensor.setRangingMode(eMatrix_8X8) != 0) {
    return false;
  }

  initialised = true;
  return true;
}

bool readFrame(Frame& frame) {
  if (!initialised) {
    return false;
  }

  return sensor.getAllData(frame) == 0;
}

bool isInitialised() {
  return initialised;
}

}  // namespace hd_raw_tof8x8
