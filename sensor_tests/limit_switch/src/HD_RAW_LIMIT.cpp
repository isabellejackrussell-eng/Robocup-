#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

// Change this value to test a different SX1509 AIO input (0-15).
const byte LIMIT_SWITCH_PIN = 0;

const byte SX1509_ADDRESS = 0x3E;
const unsigned long DEBOUNCE_MS = 30;

SX1509 io;

int stableSwitchState = HIGH;
int lastRawSwitchState = HIGH;
unsigned long lastStateChangeTime = 0;

void printLimitSwitchState(int state)
{
    // INPUT_PULLUP makes a switch connected to GND active-low.
    if (state == LOW)
    {
        Serial.println("LIMIT SWITCH: PRESSED");
    }
    else
    {
        Serial.println("LIMIT SWITCH: NOT PRESSED");
    }
}

void setup()
{
    // USB Serial Monitor output. This is separate from smart-servo Serial7.
    Serial.begin(9600);

    // Start the Teensy I2C0 bus connected to the CPU board's RAW I2C0 port.
    Wire.begin();

    if (!io.begin(SX1509_ADDRESS))
    {
        Serial.println("ERROR: Failed to communicate with SX1509");

        // Stop here because reads are not meaningful without the expander.
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("SX1509 connected successfully");

    // The internal pull-up holds the input HIGH; pressing the switch connects
    // it to GND and produces a LOW reading.
    io.pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

    // Capture and report the initial switch state once at startup.
    stableSwitchState = io.digitalRead(LIMIT_SWITCH_PIN);
    lastRawSwitchState = stableSwitchState;
    lastStateChangeTime = millis();
    printLimitSwitchState(stableSwitchState);
}

void loop()
{
    const int rawSwitchState = io.digitalRead(LIMIT_SWITCH_PIN);

    // Restart the debounce timer whenever the raw contact changes.
    if (rawSwitchState != lastRawSwitchState)
    {
        lastRawSwitchState = rawSwitchState;
        lastStateChangeTime = millis();
    }

    // Accept and print a new state only after it has remained stable for 30 ms.
    if (rawSwitchState != stableSwitchState &&
        millis() - lastStateChangeTime >= DEBOUNCE_MS)
    {
        stableSwitchState = rawSwitchState;
        printLimitSwitchState(stableSwitchState);
    }

    delay(1);
}
