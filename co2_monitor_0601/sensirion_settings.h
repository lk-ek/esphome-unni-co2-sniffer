// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "ble_options.h"
#if UNNI_BLE_ENABLED && !defined(USE_HOST)

#include "esphome/components/esp32_ble_server/ble_server.h"
#include <string>
#include <esp_gatts_api.h>

namespace esphome {
namespace co2_monitor_0601 {

void sensirion_settings_configure_gatt(esp32_ble_server::BLEServer *server, const std::string &default_name);
void sensirion_settings_loop();
bool sensirion_settings_flush();
bool sensirion_settings_advertise_data_enabled();
void sensirion_settings_set_advertise_data_enabled(bool enabled);
// UUID 0x81FE is the SHT43 DemoBoard IsLogEnabled setting. The Unni bridge
// retains it as an experimental direct-BLE alias for HA/WiFi disable.
// Sensirion does not list 0x81FE for DIY/MyCO2 gadgets, so MyAmbience is not
// expected to render it in the production gadget UI.
// false = WiFi/HA enabled, true = WiFi/HA disabled.
bool sensirion_settings_ha_disabled();
void sensirion_settings_set_ha_disabled(bool disabled);
void sensirion_settings_gatts_event_handler(esp_gatts_cb_event_t event,
                                            esp_gatt_if_t gatts_if,
                                            esp_ble_gatts_cb_param_t *param);

}  // namespace co2_monitor_0601
}  // namespace esphome
#endif
