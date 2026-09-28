#include "hd_raw_bluetooth.h"

namespace hd_raw_bluetooth {
namespace {

bool initialised = false;

}  // namespace

bool initialise(uint32_t baudRate) {
  Serial2.begin(baudRate);
  initialised = true;
  return true;
}

bool isInitialised() {
  return initialised;
}

int available() {
  return initialised ? Serial2.available() : 0;
}

bool readByte(char& value) {
  if (!initialised || Serial2.available() <= 0) {
    return false;
  }

  value = static_cast<char>(Serial2.read());
  return true;
}

size_t sendBytes(const uint8_t* data, size_t length) {
  if (!initialised || data == nullptr || length == 0) {
    return 0;
  }

  return Serial2.write(data, length);
}

size_t sendText(const char* text) {
  if (!initialised || text == nullptr) {
    return 0;
  }

  return Serial2.print(text);
}

size_t sendLine(const char* text) {
  if (!initialised || text == nullptr) {
    return 0;
  }

  return Serial2.println(text);
}

void flush() {
  if (initialised) {
    Serial2.flush();
  }
}

}  // namespace hd_raw_bluetooth
