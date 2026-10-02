#include "hd_move_servoArm.h"

#include <Servo.h>

namespace hd_move_servoArm {
namespace {

Servo armServo;
bool initialised = false;
uint8_t commandedAngleDegrees = kInitialAngleDegrees;

constexpr int kMinimumPulseWidthUs = 544;
constexpr int kMaximumPulseWidthUs = 2400;

}  // namespace

bool initialise(uint8_t initialAngleDegrees) {
    armServo.attach(
        kServoPin, kMinimumPulseWidthUs, kMaximumPulseWidthUs);
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
    // Servo.write() clamps degree commands above 180. Map the arm's tested
    // 0..200 degree command range explicitly onto its configured pulse range.
    const int pulseWidthUs = map(
        commandedAngleDegrees,
        kMinimumAngleDegrees,
        kMaximumAngleDegrees,
        kMinimumPulseWidthUs,
        kMaximumPulseWidthUs);
    armServo.writeMicroseconds(pulseWidthUs);
    return true;
}

bool isInitialised() {
    return initialised;
}

uint8_t getCommandedAngle() {
    return commandedAngleDegrees;
}

}  // namespace hd_move_servoArm
