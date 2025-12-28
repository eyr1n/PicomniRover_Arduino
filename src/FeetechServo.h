#pragma once

#include <cstddef>
#include <cstdint>

#include "FeetechBus.h"

class FeetechServo {
public:
  FeetechServo(uint8_t id, FeetechBus *bus);

  uint8_t getError();

  bool ping();
  bool readData(uint8_t addr, uint8_t *data, size_t size);
  bool writeData(uint8_t addr, const uint8_t *data, size_t size);

  bool controlMode(uint8_t value);
  bool enableTorque(uint8_t value);
  bool setPosition(int16_t value);
  bool setVelocity(int16_t value);
  bool getPosition(int16_t *value);
  bool getVelocity(int16_t *value);

private:
  uint8_t id_;
  FeetechBus *bus_;
  uint8_t error_ = 0;

  uint8_t write(uint8_t data);
  void flush();
  bool checkReply(uint8_t *data, size_t size);
};