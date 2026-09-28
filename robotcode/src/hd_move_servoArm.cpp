#include "hd_move_servoArm.h"

#include <Servo.h>

namespace hd_move_servoArm {
namespace {

Servo armServo;
bool initialised = false;
uint8_t commandedAngleDegrees = kInitialAngleDegrees;

}  // namespace

bool initialise(uint8_t initialAngleDegrees) {
    armServo.attach(kServoPin);
    initialised = armServo.attached();

    if (!initialised) {
        return false;
    }

    return setAngle(initialAngleDegrees);
}

bool setAngle(int angleDegrees) {
    if (!initialised) {
        return false;
    }

    angleDegrees = constrain(
        angleDegrees,
        static_cast<int>(kMinimumAngleDegrees),
        static_cast<int>(kMaximumAngleDegrees)
    );

    commandedAngleDegrees = static_cast<uint8_t>(angleDegrees);
    armServo.write(commandedAngleDegrees);
    return true;
}

bool isInitialised() {
    return initialised;
}

uint8_t getCommandedAngle() {
    return commandedAngleDegrees;
}

}  // namespace hd_move_servoArm
