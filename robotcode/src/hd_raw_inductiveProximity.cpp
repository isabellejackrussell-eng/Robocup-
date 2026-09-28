#include "hd_raw_inductiveProximity.h"

static constexpr uint8_t INDUCTIVE_SENSOR_PIN = A6;
// This sensor drives the input HIGH when its metal-detection LED is on.
static constexpr bool DETECTED_WHEN_HIGH = true;

void inductiveSensorInitialise(){
    pinMode(INDUCTIVE_SENSOR_PIN, INPUT);
    Serial.println("Inductive proximity sensor initialised");
    Serial.println("Port: A6Z");
    Serial.print("Teensy pin: ");
    Serial.println(INDUCTIVE_SENSOR_PIN);
}

bool inductiveSensorDetected(){
    int state = digitalRead(INDUCTIVE_SENSOR_PIN);

    if (DETECTED_WHEN_HIGH){
        return state == HIGH;
    }

    return state == LOW;
}

int inductiveSensorRaw(){
    return digitalRead(INDUCTIVE_SENSOR_PIN);
}
