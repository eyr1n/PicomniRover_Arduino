#pragma once

#include <btstack.h>

#include "Queue.h"

struct Command {
  float vx;
  float vy;
  float w;
};

struct Odometry {
  float x;
  float y;
  float yaw;
};

class WirelessControl {
public:
  static void begin();
  static Command getCommand();
  static void setOdometry(Odometry odom);

private:
  static Command cmd_;
  static Odometry odom_;
  static Queue<Command> cmd_queue_;
  static Queue<Odometry> odom_queue_;
  static unsigned long cmd_last_update_ms_;

  static uint16_t readCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t *buffer,
                               uint16_t buffer_size);
  static int writeCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode,
                           uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
  static void canSendNowCallback(void *context);
};
