#include <Arduino.h>
#include <Servo.h>

// ============================================================
// MOTOR HARDWARE — your pins
// ============================================================
const int LEFT_PIN  = 0;   // D0
const int RIGHT_PIN = 1;   // D1

// Pulse widths (microseconds) for continuous-rotation servos / ESCs.
// Same scheme your friend used — adjust if your motors behave
// differently once you test them.
const int STOP_US         = 1500;
const int FULL_FORWARD_US = 1950;
const int FULL_REVERSE_US = 1050;

// "Power" scale used below: -MAX_POWER (full reverse) .. 0 (stop) .. MAX_POWER (full forward)
const int MAX_POWER = 450;

// Flip either of these to true if that wheel spins the wrong way
const bool LEFT_INVERTED  = false;
const bool RIGHT_INVERTED = true;

Servo motorLeft;
Servo motorRight;

// Converts a power value (-MAX_POWER..MAX_POWER) into a pulse width in microseconds
int makeMotorPulse(int motorPower)
{
    motorPower = constrain(motorPower, -MAX_POWER, MAX_POWER);

    if (motorPower == 0)
    {
        return STOP_US;
    }

    if (motorPower > 0)
    {
        return map(motorPower, 0, MAX_POWER, STOP_US, FULL_FORWARD_US);
    }
    else
    {
        return map(motorPower, 0, -MAX_POWER, STOP_US, FULL_REVERSE_US);
    }
}

// Sends power values to both motors, applying inversion and clamping/pulse conversion
void writeMotors(int leftPower, int rightPower)
{
    if (LEFT_INVERTED)  leftPower  = -leftPower;
    if (RIGHT_INVERTED) rightPower = -rightPower;

    motorLeft.writeMicroseconds(makeMotorPulse(leftPower));
    motorRight.writeMicroseconds(makeMotorPulse(rightPower));
}

void setup()
{
    Serial.begin(115200);

    motorLeft.attach(LEFT_PIN);
    motorRight.attach(RIGHT_PIN);

    // Hold at stop for a moment first — some ESCs need this to arm
    writeMotors(0, 0);
    delay(2000);
}

void loop()
{
    // Simple test pattern so you can see the wheels move.
    // Once the motors are confirmed working, replace this with your
    // real drive logic (e.g. reading commands, encoders, etc.).

    Serial.println("Forward");
    writeMotors(250, 250);
    delay(2000);

    Serial.println("Stop");
    writeMotors(0, 0);
    delay(1000);

    Serial.println("Reverse");
    writeMotors(-250, -250);
    delay(2000);

    Serial.println("Stop");
    writeMotors(0, 0);
    delay(1000);
}