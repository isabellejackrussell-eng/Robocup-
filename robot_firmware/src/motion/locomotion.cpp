#include "motion/locomotion.h"

#include <Arduino.h>
#include <Servo.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"

namespace motion {
namespace {
Servo leftMotor;
Servo rightMotor;
int commandedLeft = 0;
int commandedRight = 0;
bool initialised = false;

int makeMotorPulse(int power) {
  power = constrain(power, -config::MOTOR_MAX_POWER, config::MOTOR_MAX_POWER);
  if (power == 0) return config::MOTOR_STOP_US;
  if (power > 0) {
    return map(power, 0, config::MOTOR_MAX_POWER, config::MOTOR_STOP_US,
               config::MOTOR_FULL_FORWARD_US);
  }
  return map(power, 0, -config::MOTOR_MAX_POWER, config::MOTOR_STOP_US,
             config::MOTOR_FULL_REVERSE_US);
}
}  // namespace

void initLocomotion() {
  leftMotor.attach(config::LEFT_MOTOR_PIN);
  rightMotor.attach(config::RIGHT_MOTOR_PIN);
  initialised = true;
  stopMotion();
  debug::Log.println(F("[MOTION] Initialised; tracks stopped"));
}

void setTrackSpeeds(int leftSpeed, int rightSpeed) {
  commandedLeft = constrain(leftSpeed, -config::MOTOR_MAX_POWER, config::MOTOR_MAX_POWER);
  commandedRight = constrain(rightSpeed, -config::MOTOR_MAX_POWER, config::MOTOR_MAX_POWER);
  if (!initialised) return;

  int hardwareLeft = config::LEFT_MOTOR_INVERTED ? -commandedLeft : commandedLeft;
  int hardwareRight = config::RIGHT_MOTOR_INVERTED ? -commandedRight : commandedRight;
  leftMotor.writeMicroseconds(makeMotorPulse(hardwareLeft));
  rightMotor.writeMicroseconds(makeMotorPulse(hardwareRight));
}

void moveForward(int speed) { setTrackSpeeds(abs(speed), abs(speed)); }
void moveBackward(int speed) { setTrackSpeeds(-abs(speed), -abs(speed)); }
void turnLeft(int speed) { setTrackSpeeds(-abs(speed), abs(speed)); }
void turnRight(int speed) { setTrackSpeeds(abs(speed), -abs(speed)); }
void stopMotion() { setTrackSpeeds(0, 0); }
int getCommandedLeftSpeed() { return commandedLeft; }
int getCommandedRightSpeed() { return commandedRight; }

}  // namespace motion
