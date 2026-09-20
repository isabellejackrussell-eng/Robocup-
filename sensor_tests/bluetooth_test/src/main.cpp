#include <Arduino.h>

void setup() {
    Serial2.begin(115200);
}

void loop() {
    Serial2.println("Hello from Teensy!");
    delay(1000);
}