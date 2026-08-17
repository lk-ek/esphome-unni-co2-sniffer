// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "ble_options.h"
#if UNNI_BLE_ENABLED && UNNI_SHT43_IDENTITY_PROBE && !defined(USE_HOST)

#include "esphome/components/esp32_ble_server/ble_server.h"

namespace esphome {
namespace co2_monitor_0601 {

void sensirion_sht43_probe_configure_serial_gatt(esp32_ble_server::BLEServer *server);
void sensirion_sht43_probe_configure_gatt(esp32_ble_server::BLEServer *server);
void sensirion_sht43_probe_set_temperature_humidity(float temperature_c, float humidity_percent);

}  // namespace co2_monitor_0601
}  // namespace esphome
#endif
