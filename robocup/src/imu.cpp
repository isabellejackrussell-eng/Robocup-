#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void imu_init()
{
    if (!bno.begin())
    {
        Serial.println("[IMU] BNO055 not detected!");
        return;
    }

    Serial.println("[IMU] BNO055 initialised");

    delay(1000);
}

void imu_test()
{
    sensors_event_t orientationData;
    sensors_event_t angVelocityData;
    sensors_event_t linearAccelData;

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    Serial.print("[IMU] Heading: ");
    Serial.print(orientationData.orientation.x);

    Serial.print("  Roll: ");
    Serial.print(orientationData.orientation.y);

    Serial.print("  Pitch: ");
    Serial.print(orientationData.orientation.z);

    Serial.print(" | Gyro Z: ");
    Serial.print(angVelocityData.gyro.z);

    Serial.print(" | Accel X: ");
    Serial.println(linearAccelData.acceleration.x);

    delay(100);
}