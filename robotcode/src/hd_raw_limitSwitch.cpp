#include "hd_raw_limitSwitch.h"

#include <Wire.h>
#include <SparkFunSX1509.h>

namespace hd_raw_limitSwitch {
namespace {

SX1509 expander;
bool initialised = false;

}  // namespace

bool initialise() {
    Wire.begin();

    initialised = expander.begin(kExpanderI2cAddress);
    if (!initialised) {
        return false;
    }

    // The switch connects AIO6 to ground when pressed.
    expander.pinMode(kLimitSwitchPin, INPUT_PULLUP);
    return true;
}

bool read(Reading &reading) {
    reading = {false, HIGH, millis(), false};

    if (!initialised) {
        return false;
    }

    reading.rawState = expander.digitalRead(kLimitSwitchPin);
    reading.pressed = reading.rawState == LOW;
    reading.timestampMs = millis();
    reading.valid = true;
    return true;
}

bool isInitialised() {
    return initialised;
}

}  // namespace hd_raw_limitSwitch
