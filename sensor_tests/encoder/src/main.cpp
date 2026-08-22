#include <Arduino.h>
#include <Servo.h>

// ============================================================
// MOTOR HARDWARE
// ============================================================
const int LEFT_PIN  = 0;   // D0
const int RIGHT_PIN = 1;   // D1

const int STOP_US         = 1500;
const int FULL_FORWARD_US = 1950;
const int FULL_REVERSE_US = 1050;

const int MAX_POWER = 450;

const bool LEFT_INVERTED  = false;
const bool RIGHT_INVERTED = true;

Servo motorLeft;
Servo motorRight;

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

void writeMotors(int leftPower, int rightPower)
{
    if (LEFT_INVERTED)  leftPower  = -leftPower;
    if (RIGHT_INVERTED) rightPower = -rightPower;

    motorLeft.writeMicroseconds(makeMotorPulse(leftPower));
    motorRight.writeMicroseconds(makeMotorPulse(rightPower));
}

// ============================================================
// ENCODERS — your code, pins as given (D2/D3 and D4/D5)
// ============================================================
enum PinAssignments {
    encoder1PinA = 2,
    encoder1PinB = 3,

    encoder2PinA = 4,
    encoder2PinB = 5,
};

// Changed from "unsigned int" to "volatile long": with unsigned counters,
// a reverse rotation past zero wraps around to a huge number instead of
// going negative, which makes direction hard to read back.
volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;

boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;

// Interrupt on A changing state
void doEncoder1A()
{
    A_set1 = digitalRead(encoder1PinA) == HIGH;
    encoderPos1 += (A_set1 != B_set1) ? +1 : -1;

    B_set1 = digitalRead(encoder1PinB) == HIGH;
    encoderPos1 += (A_set1 == B_set1) ? +1 : -1;
}

// Interrupt on A changing state
void doEncoder2A()
{
    A_set2 = digitalRead(encoder2PinA) == HIGH;
    encoderPos2 += (A_set2 != B_set2) ? +1 : -1;

    B_set2 = digitalRead(encoder2PinB) == HIGH;
    encoderPos2 += (A_set2 == B_set2) ? +1 : -1;
}

// ============================================================
// TEST SEQUENCE — drives forward / stop / reverse / stop on a loop
// while printing live encoder counts, so you can watch the counts
// react to the motors moving. Non-blocking (millis-based) so the
// encoder printout keeps updating during the drive/pause phases
// instead of freezing during a delay().
// ============================================================
enum TestPhase { PHASE_FORWARD, PHASE_STOP1, PHASE_REVERSE, PHASE_STOP2 };
TestPhase phase = PHASE_FORWARD;
unsigned long phaseStart = 0;

const unsigned long DRIVE_MS = 2000;
const unsigned long PAUSE_MS = 1000;

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 200;

void setup()
{
    Serial.begin(115200);

    motorLeft.attach(LEFT_PIN);
    motorRight.attach(RIGHT_PIN);
    writeMotors(0, 0);
    delay(2000); // let ESCs arm at stop before doing anything

    pinMode(encoder1PinA, INPUT);
    pinMode(encoder1PinB, INPUT);
    pinMode(encoder2PinA, INPUT);
    pinMode(encoder2PinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);

    phaseStart = millis();
}

void loop()
{
    unsigned long now = millis();

    // --- advance the motor test sequence ---
    switch (phase)
    {
        case PHASE_FORWARD:
            writeMotors(250, 250);
            if (now - phaseStart >= DRIVE_MS) { phase = PHASE_STOP1; phaseStart = now; }
            break;

        case PHASE_STOP1:
            writeMotors(0, 0);
            if (now - phaseStart >= PAUSE_MS) { phase = PHASE_REVERSE; phaseStart = now; }
            break;

        case PHASE_REVERSE:
            writeMotors(-250, -250);
            if (now - phaseStart >= DRIVE_MS) { phase = PHASE_STOP2; phaseStart = now; }
            break;

        case PHASE_STOP2:
            writeMotors(0, 0);
            if (now - phaseStart >= PAUSE_MS) { phase = PHASE_FORWARD; phaseStart = now; }
            break;
    }

    // --- report encoder counts periodically ---
    if (now - lastPrint >= PRINT_INTERVAL_MS)
    {
        lastPrint = now;
        Serial.print("Encoder1: ");
        Serial.print(encoderPos1);
        Serial.print("   Encoder2: ");
        Serial.println(encoderPos2);
    }
}