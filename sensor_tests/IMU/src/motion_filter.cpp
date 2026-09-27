#include "motion_filter.h"

#include <math.h>

namespace
{
constexpr float ORIENTATION_ALPHA = 0.18f;
constexpr float ACCELERATION_ALPHA = 0.22f;
constexpr float ACCELERATION_DEADBAND = 0.035f;       // m/s^2
constexpr float STATIONARY_ACCELERATION_LIMIT = 0.12f; // m/s^2
constexpr float STATIONARY_GYRO_LIMIT = 0.06f;         // rad/s
constexpr float BIAS_ADAPTATION_ALPHA = 0.005f;
}

void MotionFilter::begin()
{
    state_ = MotionEstimate{};
    accelerationBias_ = imu::Vector<3>();
    filteredAcceleration_ = imu::Vector<3>();
    previousAcceleration_ = imu::Vector<3>();
    velocity_ = imu::Vector<3>();
    previousSpeed_ = 0.0f;
    biasSamples_ = 0;
    stationarySamples_ = 0;
    orientationInitialised_ = false;
}

void MotionFilter::update(const imu::Vector<3> &linearAccelerationBody,
                          const imu::Vector<3> &gyroscope,
                          const imu::Quaternion &orientation,
                          float headingDeg,
                          float rollDeg,
                          float pitchDeg,
                          float deltaSeconds)
{
    if (!orientationInitialised_)
    {
        state_.headingDeg = headingDeg;
        state_.rollDeg = rollDeg;
        state_.pitchDeg = pitchDeg;
        orientationInitialised_ = true;
    }
    else
    {
        state_.headingDeg = smoothAngle(state_.headingDeg, headingDeg, ORIENTATION_ALPHA);
        state_.rollDeg = smoothAngle(state_.rollDeg, rollDeg, ORIENTATION_ALPHA);
        state_.pitchDeg = smoothAngle(state_.pitchDeg, pitchDeg, ORIENTATION_ALPHA);
        if (state_.rollDeg > 180.0f)
        {
            state_.rollDeg -= 360.0f;
        }
        if (state_.pitchDeg > 180.0f)
        {
            state_.pitchDeg -= 360.0f;
        }
    }

    // BNO055 linear acceleration has gravity removed but is expressed in the
    // sensor frame. Rotate it into a fixed world frame before integration.
    const imu::Vector<3> worldAcceleration = orientation.rotateVector(linearAccelerationBody);

    // Learn the remaining acceleration offset during the first two seconds.
    // The robot must be stationary during this period.
    if (biasSamples_ < BIAS_SAMPLE_COUNT)
    {
        accelerationBias_ = accelerationBias_ + worldAcceleration;
        ++biasSamples_;

        if (biasSamples_ == BIAS_SAMPLE_COUNT)
        {
            accelerationBias_ = accelerationBias_ / static_cast<double>(BIAS_SAMPLE_COUNT);
            state_.calibrated = true;
        }
        return;
    }

    if (deltaSeconds <= 0.0f || deltaSeconds > 0.1f)
    {
        return;
    }

    imu::Vector<3> correctedAcceleration = worldAcceleration - accelerationBias_;
    filteredAcceleration_ = filteredAcceleration_ * (1.0f - ACCELERATION_ALPHA) +
                            correctedAcceleration * ACCELERATION_ALPHA;

    const float accelerationMagnitude = sqrtf(
        filteredAcceleration_.x() * filteredAcceleration_.x() +
        filteredAcceleration_.y() * filteredAcceleration_.y() +
        filteredAcceleration_.z() * filteredAcceleration_.z());
    const float gyroMagnitude = sqrtf(
        gyroscope.x() * gyroscope.x() +
        gyroscope.y() * gyroscope.y() +
        gyroscope.z() * gyroscope.z());

    const bool looksStationary = accelerationMagnitude < STATIONARY_ACCELERATION_LIMIT &&
                                 gyroMagnitude < STATIONARY_GYRO_LIMIT;
    if (looksStationary)
    {
        if (stationarySamples_ < STATIONARY_SAMPLE_COUNT)
        {
            ++stationarySamples_;
        }
    }
    else
    {
        stationarySamples_ = 0;
    }

    state_.stationary = stationarySamples_ >= STATIONARY_SAMPLE_COUNT;
    if (state_.stationary)
    {
        // Zero-velocity update: this is the main defence against double-
        // integration drift when the robot is stopped.
        velocity_ = imu::Vector<3>();
        previousSpeed_ = 0.0f;
        state_.speedMetresPerSecond = 0.0f;
        filteredAcceleration_ = imu::Vector<3>();
        previousAcceleration_ = imu::Vector<3>();

        // Slowly follow temperature-dependent sensor bias only while stopped.
        accelerationBias_ = accelerationBias_ * (1.0f - BIAS_ADAPTATION_ALPHA) +
                            worldAcceleration * BIAS_ADAPTATION_ALPHA;
        return;
    }

    filteredAcceleration_.x() = applyDeadband(filteredAcceleration_.x(), ACCELERATION_DEADBAND);
    filteredAcceleration_.y() = applyDeadband(filteredAcceleration_.y(), ACCELERATION_DEADBAND);

    // Trapezoidal integration on the floor plane. Vertical position is not
    // integrated because it is especially unstable without another sensor.
    velocity_.x() += 0.5f * (previousAcceleration_.x() + filteredAcceleration_.x()) * deltaSeconds;
    velocity_.y() += 0.5f * (previousAcceleration_.y() + filteredAcceleration_.y()) * deltaSeconds;

    state_.positionXMetres += velocity_.x() * deltaSeconds;
    state_.positionYMetres += velocity_.y() * deltaSeconds;
    state_.speedMetresPerSecond = sqrtf(
        velocity_.x() * velocity_.x() + velocity_.y() * velocity_.y());
    state_.distanceTravelledMetres +=
        0.5f * (previousSpeed_ + state_.speedMetresPerSecond) * deltaSeconds;
    state_.displacementMetres = sqrtf(
        state_.positionXMetres * state_.positionXMetres +
        state_.positionYMetres * state_.positionYMetres);

    previousAcceleration_ = filteredAcceleration_;
    previousSpeed_ = state_.speedMetresPerSecond;
}

void MotionFilter::resetPosition()
{
    state_.positionXMetres = 0.0f;
    state_.positionYMetres = 0.0f;
    state_.displacementMetres = 0.0f;
    state_.distanceTravelledMetres = 0.0f;
    state_.speedMetresPerSecond = 0.0f;
    velocity_ = imu::Vector<3>();
    previousAcceleration_ = imu::Vector<3>();
    previousSpeed_ = 0.0f;
}

const MotionEstimate &MotionFilter::estimate() const
{
    return state_;
}

float MotionFilter::smoothAngle(float current, float target, float alpha)
{
    float difference = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
    float result = current + alpha * difference;
    return fmodf(result + 360.0f, 360.0f);
}

float MotionFilter::applyDeadband(float value, float threshold)
{
    if (fabsf(value) <= threshold)
    {
        return 0.0f;
    }
    return value > 0.0f ? value - threshold : value + threshold;
}
