#include "optical_flow.h"
#include "Bitcraze_PMW3901.h"

Bitcraze_PMW3901 flow(10);

int16_t deltaX, deltaY;

void optical_flow_init()
{
    if (!flow.begin()) {
        Serial.println("Initialization of the flow sensor failed");
        while(1) { }
    }

    Serial.println("Optical flow sensor initialised");
}

void optical_flow_test()
{
    // Get motion count since last call
    flow.readMotionCount(&deltaX, &deltaY);

    Serial.print("X: ");
    Serial.print(deltaX);

    Serial.print(", Y: ");
    Serial.print(deltaY);

    Serial.println();
    
    delay(100);
}