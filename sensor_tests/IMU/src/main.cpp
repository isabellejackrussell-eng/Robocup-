#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

#include "motion_filter.h"

namespace
{
constexpr uint8_t BNO055_ADDRESS = 0x28;
constexpr uint32_t SAMPLE_INTERVAL_MS = 20; // 50 Hz filter update
constexpr uint32_t PRINT_INTERVAL_MS = 100;

Adafruit_BNO055 bno(55, BNO055_ADDRESS);
MotionFilter motionFilter;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    if (!bno.begin())
    {
        Serial.println("ERROR: BNO055 not detected at address 0x28");
        while (true)
        {
            delay(1000);
        }
    }

    delay(1000);
    bno.setExtCrystalUse(true);
    motionFilter.begin();

    Serial.println("BNO055 motion estimator started");
    Serial.println("Keep the robot completely still for the first 2 seconds.");
}

void loop()
{
    static uint32_t previousSampleMs = millis();
    static uint32_t previousPrintMs = 0;

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - previousSampleMs;
    if (elapsedMs < SAMPLE_INTERVAL_MS)
    {
        return;
    }
    previousSampleMs = nowMs;

    const imu::Vector<3> linearAcceleration =
        bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    const imu::Vector<3> gyroscope =
        bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    const imu::Vector<3> euler =
        bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    const imu::Quaternion orientation = bno.getQuat();

    motionFilter.update(linearAcceleration,
                        gyroscope,
                        orientation,
                        euler.x(), // heading
                        euler.y(), // roll
                        euler.z(), // pitch
                        elapsedMs / 1000.0f);

    if (nowMs - previousPrintMs < PRINT_INTERVAL_MS)
    {
        return;
    }
    previousPrintMs = nowMs;

    const MotionEstimate &motion = motionFilter.estimate();
    if (!motion.calibrated)
    {
        Serial.println("CALIBRATING - keep the robot still");
        return;
    }

    Serial.print("heading_deg=");
    Serial.print(motion.headingDeg, 1);
    Serial.print(", roll_deg=");
    Serial.print(motion.rollDeg, 1);
    Serial.print(", pitch_deg=");
    Serial.print(motion.pitchDeg, 1);
    Serial.print(", distance_m=");
    Serial.print(motion.distanceTravelledMetres, 3);
    Serial.print(", displacement_m=");
    Serial.print(motion.displacementMetres, 3);
    Serial.print(", x_m=");
    Serial.print(motion.positionXMetres, 3);
    Serial.print(", y_m=");
    Serial.print(motion.positionYMetres, 3);
    Serial.print(", speed_mps=");
    Serial.print(motion.speedMetresPerSecond, 3);
    Serial.print(", stationary=");
    Serial.println(motion.stationary ? "yes" : "no");
}
