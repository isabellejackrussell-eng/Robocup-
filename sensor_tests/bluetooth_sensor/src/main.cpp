#include <Arduino.h>
#include <DFRobot_MatrixLidar.h>
#include "weight_detect.h"

const uint8_t TofAddress = 0x33;
const unsigned long BluetoothBaudRate = 115200;
const unsigned long ReadingIntervalMilliseconds = 100;
const unsigned long CalibrationDelayMilliseconds = 2000;

DFRobot_MatrixLidar_I2C tof(TofAddress);
uint16_t distances[64];

void printFrameToBluetooth();

void setup()
{
  // Serial2 is connected to the Bluetooth module.
  Serial2.begin(BluetoothBaudRate);

  while (tof.begin() != 0)
  {
    Serial2.println("begin error, retrying...");
    delay(1000);
  }
  Serial2.println("begin success");

  while (tof.setRangingMode(eMatrix_8X8) != 0)
  {
    Serial2.println("failed to set 8x8 mode, retrying...");
    delay(1000);
  }
  Serial2.println("init success, starting readings...");
  Serial2.println("Keep the sensor pointed at an empty scene.");
  Serial2.println("Calibrating background in 2 seconds...");

  delay(CalibrationDelayMilliseconds);
  tof.getAllData(distances);
  calibrateBackground(distances);

  Serial2.println("Background calibrated");
  Serial2.println("Send 'c' at any time to recalibrate.");
}

void loop()
{
  if (Serial2.available() && Serial2.read() == 'c')
  {
    tof.getAllData(distances);
    calibrateBackground(distances);
    Serial2.println("Background calibrated");
  }

  tof.getAllData(distances);
  printFrameToBluetooth();

  const WeightResult result = detectWeight(distances, true, Serial2);
  printResult(result, Serial2);
  Serial2.println("------------------------------");

  delay(ReadingIntervalMilliseconds);
}

void printFrameToBluetooth()
{
  for (uint8_t row = 0; row < 8; row++)
  {
    Serial2.print('Y');
    Serial2.print(row);
    Serial2.print(": ");

    for (uint8_t column = 0; column < 8; column++)
    {
      Serial2.print(distances[row * 8 + column]);
      Serial2.print(',');
    }

    Serial2.println();
  }

  Serial2.println("------------------------------");
}
