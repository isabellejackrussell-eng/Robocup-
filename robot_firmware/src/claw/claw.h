#pragma once

#include <Arduino.h>

namespace claw {

void initClaw();
void setClawPosition(uint8_t angle);
void grab();
void release();
void update();
bool clawBusy();
bool clawFinished();
uint8_t getClawPosition();

}  // namespace claw

