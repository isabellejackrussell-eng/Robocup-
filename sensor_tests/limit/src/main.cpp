#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3E;
const byte LIMIT_SWITCH_PIN = 6; // SX1509 AIO6

SX1509 io;

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    if (!io.begin(SX1509_ADDRESS))
    {
        Serial.println("ERROR: SX1509 not found at address 0x3E");
        while (true)
        {
            delay(1000);
        }
    }

    // The switch should connect AIO6 to GND when pressed.
    // The pull-up keeps the input HIGH while the switch is open.
    io.pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

    Serial.println("Reading limit switch on SX1509 AIO6");
}

void loop()
{
    const int switchState = io.digitalRead(LIMIT_SWITCH_PIN);

    Serial.print("AIO6: ");
    Serial.print(switchState);
    Serial.println(switchState == LOW ? " (PRESSED)" : " (NOT PRESSED)");

    delay(100);
}
