#include "object_identification.h"

#include "tof.h"

namespace
{
struct FilterState
{
    ObjectDetection filtered;
    ObjectDetection candidate;
    uint8_t sampleCount;
};

FilterState states[OBJECT_SENSOR_SET_COUNT];

bool seesObject(uint8_t sensorIndex)
{
    if (!range_tof_is_valid(sensorIndex))
        return false;

    const uint16_t distance = range_tof_get_distance_mm(sensorIndex);
    return distance > 0 && distance <= OBJECT_MAX_DISTANCE_MM;
}

ObjectDetection classifySet(uint8_t setIndex)
{
    const uint8_t lowSensor = setIndex * 2;
    const uint8_t topSensor = lowSensor + 1;

    const bool lowSeen = seesObject(lowSensor);
    const bool topSeen = seesObject(topSensor);

    if (!lowSeen)
    {
        // A top sensor detection without a bottom detection is not one of the
        // expected weight/wall shapes.
        return {topSeen ? ObjectType::UNKNOWN : ObjectType::NONE, 0};
    }

    const uint16_t objectDistance = range_tof_get_distance_mm(lowSensor);
    return {topSeen ? ObjectType::WALL : ObjectType::WEIGHT, objectDistance};
}
} // namespace

void object_identification_reset()
{
    for (uint8_t set = 0; set < OBJECT_SENSOR_SET_COUNT; ++set)
    {
        states[set].filtered = {ObjectType::NONE, 0};
        states[set].candidate = {ObjectType::NONE, 0};
        states[set].sampleCount = 0;
    }
}

void object_identification_update()
{
    for (uint8_t set = 0; set < OBJECT_SENSOR_SET_COUNT; ++set)
    {
        const ObjectDetection reading = classifySet(set);
        FilterState &state = states[set];

        if (reading.type != state.candidate.type)
        {
            state.candidate = reading;
            state.sampleCount = 1;
        }
        else
        {
            state.candidate.distanceMm = reading.distanceMm;
            if (state.sampleCount < OBJECT_STABLE_SAMPLE_COUNT)
                ++state.sampleCount;
        }

        if (state.sampleCount >= OBJECT_STABLE_SAMPLE_COUNT)
            state.filtered = state.candidate;
    }
}

ObjectDetection object_identification_get(uint8_t setIndex)
{
    if (setIndex >= OBJECT_SENSOR_SET_COUNT)
        return {ObjectType::UNKNOWN, 0};

    return states[setIndex].filtered;
}

const char *object_type_name(ObjectType type)
{
    switch (type)
    {
    case ObjectType::NONE:
        return "NONE";
    case ObjectType::WEIGHT:
        return "WEIGHT";
    case ObjectType::WALL:
        return "WALL";
    default:
        return "UNKNOWN";
    }
}

void object_identification_print()
{
    for (uint8_t set = 0; set < OBJECT_SENSOR_SET_COUNT; ++set)
    {
        const ObjectDetection detection = object_identification_get(set);

        Serial.print("Set ");
        Serial.print(set);
        Serial.print(": ");
        Serial.print(object_type_name(detection.type));

        if (detection.type != ObjectType::NONE && detection.distanceMm > 0)
        {
            Serial.print(" at ");
            Serial.print(detection.distanceMm);
            Serial.print("mm");
        }

        if (set + 1 < OBJECT_SENSOR_SET_COUNT)
            Serial.print("   ");
    }

    Serial.println();
}
