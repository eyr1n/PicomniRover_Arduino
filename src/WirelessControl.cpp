#ifdef ENABLE_CLASSIC

#include <cstdint>
#include <cstring>

#include <Arduino.h>

#include "WirelessControl.h"
#include "wireless_control.gatt.h"

constexpr unsigned long WIRELESS_CONTROL_CMD_TIMEOUT_MS = 1000;

// clang-format off
static uint8_t advertising_data[] = {
    2, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    8, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', 'm', 'n', 'i',
    17, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, 0xC5, 0x4B, 0x83, 0x0C, 0xD3, 0xB6, 0xE2, 0xB5, 0x8E, 0x48, 0x17, 0x80, 0x59, 0x1C, 0x32, 0x69,
};
static uint8_t service_uuid[] = {0x69, 0x32, 0x1C, 0x59, 0x80, 0x17, 0x48, 0x8E, 0xB5, 0xE2, 0xB6, 0xD3, 0x0C, 0x83, 0x4B, 0xC5};
static uint8_t characteristic_uuid[] = {0x87, 0xBC, 0x2D, 0xC5, 0x22, 0x07, 0x40, 0x8D, 0x99, 0xF6, 0x3D, 0x35, 0x57, 0x3C, 0x44, 0x72};
// clang-format on

static btstack_context_callback_registration_t odom_callback;
static att_service_handler_t wireless_control_service;

static uint16_t client_configuration;
static hci_con_handle_t client_configuration_connection;

static uint8_t odom_buf[sizeof(Odometry)];

Command WirelessControl::cmd_;
Odometry WirelessControl::odom_;
Queue<Command> WirelessControl::cmd_queue_;
Queue<Odometry> WirelessControl::odom_queue_;
unsigned long WirelessControl::cmd_last_update_ms_;

void WirelessControl::begin() {
  cmd_queue_.init(1);
  odom_queue_.init(1);

  __lockBluetooth();

  l2cap_init();
  sm_init();
  att_server_init(profile_data, nullptr, nullptr);

  wireless_control_service.start_handle = ATT_SERVICE_69321C59_8017_488E_B5E2_B6D30C834BC5_START_HANDLE;
  wireless_control_service.end_handle = ATT_SERVICE_69321C59_8017_488E_B5E2_B6D30C834BC5_END_HANDLE;
  wireless_control_service.read_callback = WirelessControl::readCallback;
  wireless_control_service.write_callback = WirelessControl::writeCallback;
  att_server_register_service_handler(&wireless_control_service);

  // advertise
  uint16_t adv_int_min = 0x0030;
  uint16_t adv_int_max = 0x0030;
  uint8_t adv_type = 0;
  bd_addr_t null_addr;
  std::memset(null_addr, 0, 6);
  gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
  gap_advertisements_set_data(sizeof(advertising_data), advertising_data);
  gap_advertisements_enable(1);

  hci_power_control(HCI_POWER_ON);

  __unlockBluetooth();
}

Command WirelessControl::getCommand() {
  if (static_cast<unsigned long>(millis() - cmd_last_update_ms_) >= WIRELESS_CONTROL_CMD_TIMEOUT_MS) {
    return {};
  }
  if (auto cmd = cmd_queue_.try_remove()) {
    cmd_ = *cmd;
  }
  return cmd_;
}

void WirelessControl::setOdometry(Odometry odom) {
  if (!odom_queue_.try_add(odom)) {
    odom_queue_.try_remove();
    odom_queue_.try_add(odom);
  }
}

uint16_t WirelessControl::readCallback(hci_con_handle_t, uint16_t attribute_handle, uint16_t offset, uint8_t *buffer,
                                       uint16_t buffer_size) {
  if (attribute_handle == ATT_CHARACTERISTIC_87BC2DC5_2207_408D_99F6_3D35573C4472_01_CLIENT_CONFIGURATION_HANDLE) {
    return att_read_callback_handle_little_endian_16(client_configuration, offset, buffer, buffer_size);
  }
  return 0;
}

int WirelessControl::writeCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode,
                                   uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
  if (transaction_mode != ATT_TRANSACTION_MODE_NONE) {
    return 0;
  }

  if (attribute_handle == ATT_CHARACTERISTIC_87BC2DC5_2207_408D_99F6_3D35573C4472_01_VALUE_HANDLE) {
    if (offset != 0) {
      return ATT_ERROR_INVALID_OFFSET;
    }
    if (buffer_size != sizeof(Command)) {
      return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    }

    Command cmd;
    std::memcpy(&cmd, buffer, sizeof(cmd));
    if (!cmd_queue_.try_add(cmd)) {
      cmd_queue_.try_remove();
      cmd_queue_.try_add(cmd);
    }
    cmd_last_update_ms_ = millis();

    if (auto odom = odom_queue_.try_remove()) {
      std::memcpy(odom_buf, &*odom, sizeof(odom_buf));
    }

    // notify
    if (client_configuration != 0 && client_configuration_connection == con_handle) {
      odom_callback.callback = canSendNowCallback;
      odom_callback.context = (void *)(uintptr_t)client_configuration_connection;
      att_server_register_can_send_now_callback(&odom_callback, client_configuration_connection);
    }
  }

  if (attribute_handle == ATT_CHARACTERISTIC_87BC2DC5_2207_408D_99F6_3D35573C4472_01_CLIENT_CONFIGURATION_HANDLE) {
    client_configuration = little_endian_read_16(buffer, 0);
    client_configuration_connection = con_handle;
  }

  return 0;
}

void WirelessControl::canSendNowCallback(void *context) {
  hci_con_handle_t con_handle = (hci_con_handle_t)(uintptr_t)context;
  att_server_notify(con_handle, ATT_CHARACTERISTIC_87BC2DC5_2207_408D_99F6_3D35573C4472_01_VALUE_HANDLE, odom_buf,
                    sizeof(odom_buf));
}

#endif
