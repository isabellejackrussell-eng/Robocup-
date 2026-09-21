#include "localisation/imu.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"

namespace localisation::imu {
namespace {
Adafruit_BNO055 sensor(config::BNO055_SENSOR_ID, config::BNO055_I2C_ADDRESS);
float headingDeg = 0.0f;
float angularVelocityDegPerSecond = 0.0f;
uint32_t lastReadMs = 0;
bool ready = false;
}  // namespace

bool initIMU() {
  ready = sensor.begin();
  if (ready) {
    debug::Log.println(F("[IMU] BNO055 ready"));
  } else {
    debug::Log.println(F("[IMU] ERROR: BNO055 not detected"));
  }
  return ready;
}

bool calibrateIMU() {
  if (!ready) return false;
  uint8_t system = 0, gyro = 0, accelerometer = 0, magnetometer = 0;
  sensor.getCalibration(&system, &gyro, &accelerometer, &magnetometer);
  return gyro >= 2;
}

bool readIMU() {
  if (!ready) return false;
  sensors_event_t orientation;
  sensors_event_t gyro;
  sensor.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  sensor.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
  headingDeg = orientation.orientation.x;
  angularVelocityDegPerSecond = gyro.gyro.z * RAD_TO_DEG;
  lastReadMs = millis();
  return true;
}

float getHeading() { return headingDeg; }
float getAngularVelocity() { return angularVelocityDegPerSecond; }
bool healthy() { return ready && millis() - lastReadMs <= config::SENSOR_STALE_TIMEOUT_MS; }

}  // namespace localisation::imu

