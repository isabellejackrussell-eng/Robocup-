#include "hd_raw_ultrasonic.h"

namespace hd_raw_ultrasonic {
namespace {

constexpr uint8_t kTriggerPins[kSensorCount] = {3, 5};
constexpr uint8_t kEchoPins[kSensorCount] = {2, 4};

}  // namespace

void initialise() {
  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    pinMode(kTriggerPins[sensorIndex], OUTPUT);
    digitalWrite(kTriggerPins[sensorIndex], LOW);
    pinMode(kEchoPins[sensorIndex], INPUT);
  }
}

bool readSensor(Sensor sensor, Reading& reading) {
  const uint8_t sensorIndex = static_cast<uint8_t>(sensor);
  reading = {0, 0, millis(), false, false};

  if (sensorIndex >= kSensorCount) {
    return false;
  }

  const uint8_t triggerPin = kTriggerPins[sensorIndex];
  const uint8_t echoPin = kEchoPins[sensorIndex];

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  reading.echoDurationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  reading.timestampMs = millis();
  reading.timedOut = reading.echoDurationUs == 0;
  reading.valid = !reading.timedOut;

  if (reading.valid) {
    reading.distanceCm = microsecondsToCentimetres(reading.echoDurationUs);
  }

  return reading.valid;
}

void readAll(Readings& readings) {
  readSensor(Sensor::a, readings[0]);
  readSensor(Sensor::b, readings[1]);
}

uint16_t microsecondsToCentimetres(uint32_t microseconds) {
  return microseconds / 29 / 2;
}

}  // namespace hd_raw_ultrasonic
