#include "schedulers/scheduler_common.h"

#include "claw/claw.h"
#include "debug/debug_port.h"
#include "debug/robot_debug.h"
#include "localisation/encoders.h"
#include "localisation/home_vector.h"
#include "localisation/imu.h"
#include "localisation/optical_flow.h"
#include "localisation/pose_estimator.h"
#include "motion/locomotion.h"
#include "navigation/navigation.h"
#include "perception/capture_detection.h"
#include "perception/object_validation.h"
#include "perception/sensor_state.h"
#include "perception/wall_detection.h"
#include "perception/weight_detection.h"
#include "sensors/capture_sensors.h"
#include "sensors/range_tof.h"
#include "sensors/tof8x8.h"
#include "sensors/wall_sensors.h"
#include "state/robot_state.h"

namespace schedulers::common {
namespace {
void handleCommand(char command) {
  switch (command) {
    case 'c':
    case 'C':
      if (sensors::tof8x8::calibrate8x8()) {
        perception::weight::calibrateBackground(sensors::tof8x8::getCalibrationFrame());
        debug::Log.println(F("[COMMAND] 8x8 calibration applied"));
      } else {
        debug::Log.println(F("[COMMAND] Calibration failed: no 8x8 frame"));
      }
      break;
    case 'p':
    case 'P':
      debug::print8x8Grid();
      break;
    case 'g':
    case 'G':
      state::startAutonomous();
      debug::Log.println(F("[COMMAND] Autonomous test started"));
      break;
    case 'x':
    case 'X':
      state::stopAutonomous();
      motion::stopMotion();
      debug::Log.println(F("[COMMAND] STOPPED"));
      break;
    case 'r':
    case 'R':
      state::requestReturnHome();
      debug::Log.println(F("[COMMAND] Return-home requested"));
      break;
    case 'h':
    case 'H':
    case '?':
      debug::printHelp();
      break;
    case '\n':
    case '\r':
      break;
    default:
      debug::Log.print(F("[COMMAND] Unknown: "));
      debug::Log.println(command);
      debug::printHelp();
      break;
  }
}
}  // namespace

void initSensorSuite() {
  sensors::range_tof::initRangeToFSensors();
  sensors::tof8x8::init8x8();
  sensors::wall::initWallSensors();
  sensors::capture::initCaptureSensors();
}

void initLocalisationSuite() {
  localisation::imu::initIMU();
  localisation::optical_flow::initOpticalFlow();
  localisation::encoders::initEncoders();
  localisation::pose::initialisePose();
}

void serviceFastTasks() {
  sensors::wall::serviceUltrasonics();
  claw::update();
  pollDebugCommands();
}

void readSensorSuite() {
  sensors::tof8x8::read8x8();
  sensors::range_tof::readAll();
  sensors::wall::readAllWallSensors();
  sensors::capture::readCaptureSensors();
}

void processPerception() {
  perception::weight::process8x8Frame(sensors::tof8x8::get8x8Frame());
  perception::wall::updateWallState(sensors::wall::getWallSensorRaw());
  perception::validation::validateWeightCandidate(perception::weight::getWeightData(),
                                                   perception::wall::getWallState());
  perception::capture::filterCaptureReadings(sensors::capture::getCaptureSensorRaw());
  perception::updateSensorState();
}

void readLocalisation() {
  localisation::imu::readIMU();
  localisation::optical_flow::readOpticalFlow();
  localisation::encoders::updateEncoders();
}

void updateLocalisation() {
  localisation::pose::updatePose();
  localisation::home::calculateHomeVector(localisation::pose::getPose());
}

void applyNavigationCommand() {
  const navigation::MotionCommand command = navigation::calculateMotionCommand();
  if (command.stop || state::getRobotMode() == state::RobotMode::IDLE ||
      state::getRobotMode() == state::RobotMode::CAPTURING ||
      state::getRobotMode() == state::RobotMode::ERROR) {
    motion::stopMotion();
  } else {
    motion::setTrackSpeeds(static_cast<int>(command.leftSpeed),
                           static_cast<int>(command.rightSpeed));
  }
}

void pollDebugCommands() {
  while (Serial.available()) handleCommand(static_cast<char>(Serial.read()));
  while (Serial2.available()) handleCommand(static_cast<char>(Serial2.read()));
}

void printAllDebug(bool includeGrid) {
  debug::printSensorDebug();
  if (includeGrid) debug::print8x8Grid();
  debug::printPoseDebug();
  debug::printNavigationDebug();
}

}  // namespace schedulers::common
