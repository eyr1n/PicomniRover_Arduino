#pragma once

#include <cstddef>
#include <cstdint>

#include <Arduino.h>
#include <hardware/pio.h>

#include "Queue.h"

class FeetechBus {
public:
  FeetechBus(pin_size_t pin = 27);
  void begin(uint32_t baudrate = 1000000, size_t rxQueueSize = 32);
  size_t available();
  uint8_t read();
  void write(uint8_t data);

private:
  pin_size_t pin_;
  PIO pio_;
  uint tx_sm_;
  uint rx_sm_;
  Queue<uint8_t> rx_queue_;

  static void rxIrqHandler();
};
