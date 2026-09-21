#include "debug/robot_debug.h"

#include <math.h>

#include "debug/debug_port.h"
#include "localisation/encoders.h"
#include "localisation/home_vector.h"
#include "localisation/imu.h"
#include "localisation/optical_flow.h"
#include "localisation/pose_estimator.h"
#include "navigation/navigation.h"
#include "perception/capture_detection.h"
#include "perception/object_validation.h"
#include "perception/sensor_state.h"
#include "perception/wall_detection.h"
#include "perception/weight_detection.h"
#include "sensors/capture_sensors.h"
#include "sensors/tof8x8.h"
#include "sensors/wall_sensors.h"
#include "state/robot_state.h"

namespace debug {
namespace {
void printDistance(const DistanceReading& reading) {
  if (reading.valid) {
    Log.print(reading.distanceMm, 1);
    Log.print(F("mm"));
  } else {
    Log.print(F("INVALID"));
  }
}

void printFloatOrNA(float value, uint8_t digits = 1) {
  if (isnan(value)) Log.print(F("N/A"));
  else Log.print(value, digits);
}
}  // namespace

void print8x8Grid() {
  const sensors::tof8x8::Frame& frame = sensors::tof8x8::get8x8Frame();
  if (!frame.valid) {
    Log.println(F("[8X8] No valid frame"));
    return;
  }
  for (uint8_t row = 0; row < 8; ++row) {
    Log.print(F("[8X8] row "));
    Log.print(row);
    Log.print(F(":"));
    for (uint8_t column = 0; column < 8; ++column) {
      Log.print(' ');
      Log.print(frame.distanceMm[row * 8 + column]);
    }
    Log.println();
  }
}

void printSensorDebug() {
  const sensors::wall::WallSensorRaw& wallRaw = sensors::wall::getWallSensorRaw();
  const sensors::capture::CaptureSensorRaw& captureRaw = sensors::capture::getCaptureSensorRaw();
  const perception::weight::WeightData& weight = perception::weight::getWeightData();
  const perception::wall::WallState& walls = perception::wall::getWallState();
  const perception::validation::ValidationResult& validation =
      perception::validation::getValidationResult();
  const perception::capture::CaptureState& capture = perception::capture::getCaptureState();

  Log.print(F("[8X8] calibrated="));
  Log.print(perception::weight::backgroundCalibrated() ? F("YES") : F("NO"));
  Log.print(F(" candidate="));
  Log.print(weight.found ? F("YES") : F("NO"));
  Log.print(F(" distance="));
  printFloatOrNA(weight.found ? weight.distanceMm : NAN);
  Log.print(F("mm bearing="));
  printFloatOrNA(weight.found ? weight.bearingDeg : NAN);
  Log.print(F("deg cluster="));
  Log.print(weight.clusterSize);
  Log.print(F(" confidence="));
  Log.println(weight.confidence, 2);

  Log.print(F("[WALL RAW] ultrasonic L/F/R="));
  printDistance(wallRaw.ultrasonicLeft); Log.print(F(" / "));
  printDistance(wallRaw.ultrasonicFront); Log.print(F(" / "));
  printDistance(wallRaw.ultrasonicRight);
  Log.print(F(" cross L/R="));
  printDistance(wallRaw.crossToFLeft); Log.print(F(" / "));
  printDistance(wallRaw.crossToFRight); Log.println();

  Log.print(F("[WALL] filtered L/F/R="));
  printFloatOrNA(walls.wallLeftDistanceMm); Log.print(F(" / "));
  printFloatOrNA(walls.wallFrontDistanceMm); Log.print(F(" / "));
  printFloatOrNA(walls.wallRightDistanceMm);
  Log.print(F(" corner L/R="));
  Log.print(walls.possibleLeftCorner ? F("YES") : F("NO"));
  Log.print('/');
  Log.println(walls.possibleRightCorner ? F("YES") : F("NO"));

  Log.print(F("[VALIDATION] status="));
  Log.print(perception::validation::statusName(validation.status));
  Log.print(F(" wallDistance="));
  printFloatOrNA(validation.wallDistanceMm);
  Log.print(F("mm separation="));
  printFloatOrNA(validation.separationMm);
  Log.println(F("mm"));

  Log.print(F("[CAPTURE RAW] L/R="));
  printDistance(captureRaw.left); Log.print(F(" / "));
  printDistance(captureRaw.right);
  Log.print(F(" triggered L/R="));
  Log.print(capture.leftTriggered ? F("YES") : F("NO"));
  Log.print('/');
  Log.print(capture.rightTriggered ? F("YES") : F("NO"));
  Log.print(F(" BOTH required -> weight="));
  Log.println(capture.weightPresent ? F("YES") : F("NO"));
}

void printPoseDebug() {
  const localisation::pose::Pose& pose = localisation::pose::getPose();
  const localisation::home::HomeVector home = localisation::home::calculateHomeVector(pose);
  Log.print(F("[IMU] heading=")); Log.print(localisation::imu::getHeading(), 1);
  Log.print(F("deg angularVelocity=")); Log.print(localisation::imu::getAngularVelocity(), 1);
  Log.println(F("deg/s"));
  Log.print(F("[FLOW] dX/dY counts=")); Log.print(localisation::optical_flow::getDeltaXCounts());
  Log.print('/'); Log.print(localisation::optical_flow::getDeltaYCounts());
  Log.print(F(" calibrated="));
  Log.println(localisation::optical_flow::calibrated() ? F("YES") : F("NO"));
  Log.print(F("[ENCODER] ticks L/R=")); Log.print(localisation::encoders::getLeftTicks());
  Log.print('/'); Log.print(localisation::encoders::getRightTicks());
  Log.print(F(" calibrated="));
  Log.println(localisation::encoders::calibrated() ? F("YES") : F("NO"));
  Log.print(F("[POSE] x=")); Log.print(pose.xMm, 1); Log.print(F("mm y="));
  Log.print(pose.yMm, 1); Log.print(F("mm heading=")); Log.print(pose.headingDeg, 1);
  Log.print(F("deg translationCalibrated="));
  Log.println(pose.translationCalibrated ? F("YES") : F("NO"));
  Log.print(F("[HOME] distance=")); Log.print(home.distanceMm, 1);
  Log.print(F("mm relativeBearing=")); Log.print(home.bearingDeg, 1); Log.println(F("deg"));
}

void printNavigationDebug() {
  const navigation::MotionCommand& command = navigation::getMotionCommand();
  Log.print(F("[FSM] mode=")); Log.println(state::robotModeName(state::getRobotMode()));
  Log.print(F("[NAV] left=")); Log.print(command.leftSpeed, 1);
  Log.print(F(" right=")); Log.print(command.rightSpeed, 1);
  Log.print(F(" stop=")); Log.println(command.stop ? F("YES") : F("NO"));
}

void printHelp() {
  Log.println(F("[COMMAND] h=help, c=calibrate empty 8x8 scene, p=print 8x8 grid"));
  Log.println(F("[COMMAND] g=start autonomous test, x=stop, r=return home"));
}

}  // namespace debug

