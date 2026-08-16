// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "ble_options.h"
#if UNNI_BLE_ENABLED

#include "esphome/components/esp32_ble_server/ble_server.h"
#include <esp_gatts_api.h>

namespace esphome {
namespace co2_monitor_0601 {

void sensirion_settings_configure_gatt(esp32_ble_server::BLEServer *server);
void sensirion_settings_loop();
bool sensirion_settings_advertise_data_enabled();
void sensirion_settings_set_advertise_data_enabled(bool enabled);
// MyAmbience exposes UUID 0x81FE as IsLogEnabled. The Unni bridge reuses
// that standard Sensirion boolean as its BLE-accessible HA/WiFi disable flag:
// false = WiFi/HA enabled, true = WiFi/HA disabled.
bool sensirion_settings_ha_disabled();
void sensirion_settings_set_ha_disabled(bool disabled);
void sensirion_settings_gatts_event_handler(esp_gatts_cb_event_t event,
                                            esp_gatt_if_t gatts_if,
                                            esp_ble_gatts_cb_param_t *param);

}  // namespace co2_monitor_0601
}  // namespace esphome
#endif
