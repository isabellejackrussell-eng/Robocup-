#include "hd_raw_imu.h"

#include <Adafruit_BNO055.h>
#include <Wire.h>

namespace hd_raw_imu {
namespace {

Adafruit_BNO055 sensor(55, kI2cAddress);
bool initialised = false;

}  // namespace

bool initialise() {
  initialised = false;
  Wire.begin();

  if (!sensor.begin()) {
    return false;
  }

  // Match the known-working sensor test startup sequence.
  delay(1000);
  sensor.setExtCrystalUse(true);
  initialised = true;
  return true;
}

bool read(Reading& reading) {
  reading = {};
  reading.timestampMs = millis();

  if (!initialised) {
    return false;
  }

  const imu::Vector<3> linearAcceleration =
      sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  const imu::Vector<3> gyroscope =
      sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const imu::Vector<3> euler =
      sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  const imu::Quaternion orientation = sensor.getQuat();

  reading.linearAccelerationMps2 = {
      static_cast<float>(linearAcceleration.x()),
      static_cast<float>(linearAcceleration.y()),
      static_cast<float>(linearAcceleration.z()),
  };
  reading.gyroscopeRadiansPerSecond = {
      static_cast<float>(gyroscope.x()),
      static_cast<float>(gyroscope.y()),
      static_cast<float>(gyroscope.z()),
  };

  reading.headingDegrees = euler.x();
  reading.rollDegrees = euler.y();
  reading.pitchDegrees = euler.z();
  reading.orientation = {
      static_cast<float>(orientation.w()),
      static_cast<float>(orientation.x()),
      static_cast<float>(orientation.y()),
      static_cast<float>(orientation.z()),
  };

  sensor.getCalibration(
      &reading.calibration.system,
      &reading.calibration.gyroscope,
      &reading.calibration.accelerometer,
      &reading.calibration.magnetometer);
  reading.temperatureCelsius = sensor.getTemp();
  reading.timestampMs = millis();
  reading.valid = true;
  return true;
}

bool isInitialised() {
  return initialised;
}

}  // namespace hd_raw_imu
