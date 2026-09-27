#include <Arduino.h>

#include "smartServo.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SERIAL_WAIT_MS = 3000;
constexpr uint32_t IO_POWER_STARTUP_MS = 500;
constexpr uint32_t MOVE_INTERVAL_MS = 2000;

// The robot CPU board gates power to its external I/O circuitry on pin 49.
// This must be enabled before starting the Herkulex UART.
constexpr uint8_t IO_POWER_PIN = 49;

constexpr uint8_t SERVO_1_ID = 1;
constexpr uint8_t SERVO_4_ID = 4;

constexpr float TEST_ANGLES[] = {-20.0f, 0.0f, 20.0f, 0.0f};
constexpr size_t TEST_ANGLE_COUNT =
    sizeof(TEST_ANGLES) / sizeof(TEST_ANGLES[0]);

size_t nextTestAngle = 0;
uint32_t lastMoveAt = 0;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);

    // Give the USB serial monitor a chance to connect without preventing the
    // test from starting when the board is powered without a computer.
    const uint32_t waitStartedAt = millis();
    while (!Serial && millis() - waitStartedAt < SERIAL_WAIT_MS)
    {
        delay(10);
    }

    Serial.println("[TEST] Enabling CPU-board I/O power on pin 49");
    pinMode(IO_POWER_PIN, OUTPUT);
    digitalWrite(IO_POWER_PIN, HIGH);

    // Match the startup sequence from the working new_codebase branch.
    delay(IO_POWER_STARTUP_MS);

    smartServoInitialise();

    smartServoPrintStatus(SERVO_1_ID);
    smartServoPrintStatus(SERVO_4_ID);

    Serial.println("[TEST] Starting +/-20 degree movement test");
    lastMoveAt = millis();
}

void loop()
{
    smartServoUpdate();

    const uint32_t now = millis();
    if (now - lastMoveAt < MOVE_INTERVAL_MS)
    {
        return;
    }

    lastMoveAt = now;
    const float angle = TEST_ANGLES[nextTestAngle];

    Serial.print("[TEST] Moving available servos to ");
    Serial.print(angle);
    Serial.println(" degrees");

    if (smartServoIsReady(SERVO_1_ID))
    {
        setServoAngle(SERVO_1_ID, angle);
    }

    if (smartServoIsReady(SERVO_4_ID))
    {
        setServoAngle(SERVO_4_ID, angle);
    }

    nextTestAngle = (nextTestAngle + 1) % TEST_ANGLE_COUNT;
}
