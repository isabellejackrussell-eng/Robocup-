#include <Arduino.h>
#include "Bitcraze_PMW3901.h"

const int CS_PIN = 10;
Bitcraze_PMW3901 flow(CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!flow.begin()) {
    Serial.println("PMW3901 init failed - check wiring/CS pin");
    while (1) {}
  }
  Serial.println("PMW3901 ready");
}

void loop() {
  int16_t deltaX, deltaY;
  flow.readMotionCount(&deltaX, &deltaY);

  Serial.print("dX: "); Serial.print(deltaX);
  Serial.print("  dY: "); Serial.println(deltaY);

  delay(50);
}