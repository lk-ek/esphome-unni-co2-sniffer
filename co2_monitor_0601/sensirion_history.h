// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "ble_options.h"
#if UNNI_BLE_HISTORY_ENABLED && !defined(USE_HOST)

#include <cstdint>
#include <esp_gatts_api.h>
#include "esphome/components/esp32_ble_server/ble_server.h"

namespace esphome {
namespace co2_monitor_0601 {

// Bit 0: I2C capture, bit 1: RT/RH capture.
using SensirionHistoryCaptureProbe = uint8_t (*)();

void sensirion_history_setup();
void sensirion_history_loop(SensirionHistoryCaptureProbe capture_probe);
void sensirion_history_note_valid_co2_frame();
void sensirion_history_note_rtrh_cycle();
void sensirion_history_note_co2_capture(uint16_t raw_scl_edges, bool frame_error);
bool sensirion_history_flush();
void sensirion_history_configure_gatt(esp32_ble_server::BLEServer *server);
void sensirion_history_gatts_event_handler(esp_gatts_cb_event_t event,
                                           esp_gatt_if_t gatts_if,
                                           esp_ble_gatts_cb_param_t *param);

}  // namespace co2_monitor_0601
}  // namespace esphome

#endif  // UNNI_BLE_HISTORY_ENABLED
