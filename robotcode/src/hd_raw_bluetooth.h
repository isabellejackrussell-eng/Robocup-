#pragma once

#include <Arduino.h>

namespace hd_raw_bluetooth {

constexpr uint32_t kDefaultBaudRate = 115200;

// Starts the Bluetooth UART on Serial2. A true return value means the Teensy
// UART was started; it does not prove that a remote Bluetooth device is paired.
bool initialise(uint32_t baudRate = kDefaultBaudRate);

bool isInitialised();

// Number of received bytes currently waiting to be read.
int available();

// Reads one received byte without blocking. Returns false when no byte is
// available or the UART has not been initialised.
bool readByte(char& value);

// Sends bytes or text over Bluetooth. Returns the number of bytes accepted by
// the UART, or zero when the UART has not been initialised.
size_t sendBytes(const uint8_t* data, size_t length);
size_t sendText(const char* text);
size_t sendLine(const char* text);

// Waits until queued outgoing bytes have been transmitted.
void flush();

}  // namespace hd_raw_bluetooth
