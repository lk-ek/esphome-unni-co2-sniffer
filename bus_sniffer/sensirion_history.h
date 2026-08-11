// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "ble_options.h"
#if UNNI_BLE_HISTORY_ENABLED

#include <esp_gatts_api.h>
#include "esphome/components/esp32_ble_server/ble_server.h"

namespace esphome {
namespace bus_sniffer {

void sensirion_history_setup();
void sensirion_history_loop();
void sensirion_history_configure_gatt(esp32_ble_server::BLEServer *server);
void sensirion_history_gatts_event_handler(esp_gatts_cb_event_t event,
                                           esp_gatt_if_t gatts_if,
                                           esp_ble_gatts_cb_param_t *param);

}  // namespace bus_sniffer
}  // namespace esphome

#endif  // UNNI_BLE_HISTORY_ENABLED
