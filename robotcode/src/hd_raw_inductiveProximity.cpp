#include "hd_raw_inductiveProximity.h"

static constexpr uint8_t INDUCTIVE_SENSOR_PIN = A6;
static constexpr bool DETECTED_WHEN_LOW = false;

void inductiveSensorInitialise(){
    pinMode(INDUCTIVE_SENSOR_PIN, INPUT);
    Serial.println("Inductive proximity sensor initialised");
    Serial.println("Port: A6Z");
    Serial.print("Teensy pin: ");
    Serial.println(INDUCTIVE_SENSOR_PIN);
}

bool inductiveSensorDetected(){
    int state = digitalRead(INDUCTIVE_SENSOR_PIN);

    if (DETECTED_WHEN_LOW){
        return state == LOW;
    }

    return state == HIGH;
}

int inductiveSensorRaw(){
    return digitalRead(INDUCTIVE_SENSOR_PIN);
}
