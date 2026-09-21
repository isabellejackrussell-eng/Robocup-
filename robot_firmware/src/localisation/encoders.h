#pragma once

namespace localisation::encoders {

void initEncoders();
void updateEncoders();
long getLeftTicks();
long getRightTicks();
float getLeftDistance();
float getRightDistance();
bool calibrated();

}  // namespace localisation::encoders

