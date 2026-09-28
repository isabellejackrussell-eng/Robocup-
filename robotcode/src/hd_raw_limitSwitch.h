#ifndef HD_RAW_LIMIT_SWITCH_H
#define HD_RAW_LIMIT_SWITCH_H

#include <Arduino.h>

namespace hd_raw_limitSwitch {

constexpr uint8_t kExpanderI2cAddress = 0x3E;
constexpr uint8_t kLimitSwitchPin = 6;

struct Reading {
    bool pressed;
    uint8_t rawState;
    uint32_t timestampMs;
    bool valid;
};

bool initialise();
bool read(Reading &reading);
bool isInitialised();

}  // namespace hd_raw_limitSwitch

#endif
