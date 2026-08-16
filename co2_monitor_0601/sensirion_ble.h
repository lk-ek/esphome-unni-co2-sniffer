// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "ble_options.h"
#if UNNI_BLE_ENABLED

#include "sensirion_sample.h"

#include <cstdint>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>

namespace esphome {
namespace co2_monitor_0601 {

void sensirion_ble_setup();
void sensirion_ble_set_advertising_interval(uint32_t interval_ms);
void sensirion_ble_set_advertise_data_enabled(bool enabled);
void sensirion_ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void sensirion_ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_ble_gatts_cb_param_t *param);

void sensirion_ble_set_temperature_humidity(float temperature_c, float humidity_percent);
void sensirion_ble_set_co2(uint16_t ppm);
void sensirion_ble_commit_live_advertisement();

const SensirionSample &sensirion_ble_sample();
uint16_t sensirion_ble_get_device_id();

}  // namespace co2_monitor_0601
}  // namespace esphome

#endif
