#pragma once

#include <Arduino.h>
#include <utility/imumaths.h>

struct MotionEstimate
{
    float headingDeg = 0.0f;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;

    float positionXMetres = 0.0f;
    float positionYMetres = 0.0f;
    float displacementMetres = 0.0f;
    float distanceTravelledMetres = 0.0f;
    float speedMetresPerSecond = 0.0f;

    bool calibrated = false;
    bool stationary = true;
};

class MotionFilter
{
public:
    void begin();
    void update(const imu::Vector<3> &linearAccelerationBody,
                const imu::Vector<3> &gyroscope,
                const imu::Quaternion &orientation,
                float headingDeg,
                float rollDeg,
                float pitchDeg,
                float deltaSeconds);

    void resetPosition();
    const MotionEstimate &estimate() const;

private:
    static constexpr uint16_t BIAS_SAMPLE_COUNT = 100;
    static constexpr uint8_t STATIONARY_SAMPLE_COUNT = 8;

    MotionEstimate state_;
    imu::Vector<3> accelerationBias_;
    imu::Vector<3> filteredAcceleration_;
    imu::Vector<3> previousAcceleration_;
    imu::Vector<3> velocity_;

    float previousSpeed_ = 0.0f;
    uint16_t biasSamples_ = 0;
    uint8_t stationarySamples_ = 0;
    bool orientationInitialised_ = false;

    static float smoothAngle(float current, float target, float alpha);
    static float applyDeadband(float value, float threshold);
};
