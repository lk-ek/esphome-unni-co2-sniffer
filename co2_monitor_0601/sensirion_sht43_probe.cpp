// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Experimental MyAmbience identity probe. UUID topology follows the official
// Sensirion SHT43 DemoBoard firmware (BSD-3-Clause); see THIRD_PARTY_NOTICES.md.
#include "ble_options.h"
#if UNNI_BLE_ENABLED && UNNI_SHT43_IDENTITY_PROBE
#include "sensirion_sht43_probe.h"

#include "esphome/components/esp32_ble/ble.h"
#include "esphome/components/esp32_ble_server/ble_characteristic.h"
#include "esphome/components/bytebuffer/bytebuffer.h"
#include "esphome/core/log.h"


namespace esphome {
namespace co2_monitor_0601 {
namespace {
static const char *TAG = "sht43_identity";
using esp32_ble::ESPBTUUID;
using esp32_ble_server::BLECharacteristic;
using bytebuffer::ByteBuffer;

struct ProbeGatt {
  bool bound{false};
  BLECharacteristic *serial{nullptr};
  BLECharacteristic *temperature{nullptr};
  BLECharacteristic *humidity{nullptr};
} gatt;

BLECharacteristic *get_or_create(esp32_ble_server::BLEService *service,
                                 const char *uuid, uint32_t properties) {
  const auto id = ESPBTUUID::from_raw(uuid);
  auto *characteristic = service->get_characteristic(id);
  return characteristic != nullptr ? characteristic : service->create_characteristic(id, properties);
}

esp32_ble_server::BLEService *get_or_create_service(esp32_ble_server::BLEServer *server,
                                                     const char *uuid, uint16_t handles) {
  const auto id = ESPBTUUID::from_raw(uuid);
  auto *service = server->get_service(id);
  return service != nullptr ? service : server->create_service(id, false, handles);
}

void set_float(BLECharacteristic *characteristic, float value) {
  if (characteristic == nullptr) return;
  characteristic->set_value(ByteBuffer::wrap(value));
}
}  // namespace


void sensirion_sht43_probe_configure_serial_gatt(esp32_ble_server::BLEServer *server) {
  if (gatt.bound || server == nullptr) return;

  auto *sht = get_or_create_service(server, "00006000-B38D-4985-720E-0F993A68EE41", 4);
  if (sht == nullptr) {
    ESP_LOGE(TAG, "failed to create SHT43 serial-number service");
    return;
  }

  gatt.serial = get_or_create(sht, "00006001-B38D-4985-720E-0F993A68EE41",
                              BLECharacteristic::PROPERTY_READ);
  if (gatt.serial == nullptr) {
    ESP_LOGE(TAG, "failed to create SHT43 serial-number characteristic");
    return;
  }

  const uint32_t serial = 0x68430001U;
  gatt.serial->set_value(ByteBuffer::wrap(serial));
  server->enqueue_start_service(sht);
  gatt.bound = true;
  ESP_LOGI(TAG, "SHT43 serial GATT configured (0x6000/0x6001 only)");
}

void sensirion_sht43_probe_configure_serial_temperature_gatt(esp32_ble_server::BLEServer *server) {
  if (gatt.bound || server == nullptr) return;

  // Minimal valid ATT reserves for this A/B build:
  // - serial READ-only characteristic: service + declaration + value = 3 handles
  // - temperature READ|NOTIFY characteristic: service + declaration + value + CCCD = 4 handles
  auto *sht = get_or_create_service(server, "00006000-B38D-4985-720E-0F993A68EE41", 3);
  auto *temperature = get_or_create_service(server, "00002234-B38D-4985-720E-0F993A68EE41", 4);
  if (sht == nullptr || temperature == nullptr) {
    ESP_LOGE(TAG, "failed to create SHT43 serial/temperature services");
    return;
  }

  gatt.serial = get_or_create(sht, "00006001-B38D-4985-720E-0F993A68EE41",
                              BLECharacteristic::PROPERTY_READ);
  gatt.temperature = get_or_create(temperature, "00002235-B38D-4985-720E-0F993A68EE41",
                                   BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  if (gatt.serial == nullptr || gatt.temperature == nullptr) {
    ESP_LOGE(TAG, "failed to create SHT43 serial/temperature characteristics");
    return;
  }

  const uint32_t serial = 0x68430001U;
  gatt.serial->set_value(ByteBuffer::wrap(serial));
  set_float(gatt.temperature, 25.0f);

  server->enqueue_start_service(sht);
  server->enqueue_start_service(temperature);
  gatt.bound = true;
  ESP_LOGI(TAG, "SHT43 serial + temperature GATT configured with minimal handle reserves (serial=3, temperature=4)");
}

void sensirion_sht43_probe_configure_gatt(esp32_ble_server::BLEServer *server) {
  if (gatt.bound || server == nullptr) return;

  // SHT service: serial number characteristic.
  auto *sht = get_or_create_service(server, "00006000-B38D-4985-720E-0F993A68EE41", 4);
  // Dedicated temperature and humidity services used by the SHT43 DemoBoard.
  auto *temperature = get_or_create_service(server, "00002234-B38D-4985-720E-0F993A68EE41", 5);
  auto *humidity = get_or_create_service(server, "00001234-B38D-4985-720E-0F993A68EE41", 5);
  if (sht == nullptr || temperature == nullptr || humidity == nullptr) {
    ESP_LOGE(TAG, "failed to create SHT43 identity-probe services");
    return;
  }

  gatt.serial = get_or_create(sht, "00006001-B38D-4985-720E-0F993A68EE41",
                              BLECharacteristic::PROPERTY_READ);
  gatt.temperature = get_or_create(temperature, "00002235-B38D-4985-720E-0F993A68EE41",
                                   BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gatt.humidity = get_or_create(humidity, "00001235-B38D-4985-720E-0F993A68EE41",
                                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  if (gatt.serial == nullptr || gatt.temperature == nullptr || gatt.humidity == nullptr) {
    ESP_LOGE(TAG, "failed to create complete SHT43 identity-probe GATT topology");
    return;
  }

  // A plausible fixed serial is sufficient for classification probing; the
  // advertisement uses a distinct test BLE identity/device ID (0x6843).
  const uint32_t serial = 0x68430001U;
  gatt.serial->set_value(ByteBuffer::wrap(serial));
  set_float(gatt.temperature, 25.0f);
  set_float(gatt.humidity, 50.0f);

  server->enqueue_start_service(sht);
  server->enqueue_start_service(temperature);
  server->enqueue_start_service(humidity);
  gatt.bound = true;
  ESP_LOGI(TAG, "SHT43 identity-probe GATT configured (SHT/T/RH services; expanded handle reserves)");
}

void sensirion_sht43_probe_set_temperature_humidity(float temperature_c, float humidity_percent) {
  if (!gatt.bound) return;
  set_float(gatt.temperature, temperature_c);
  set_float(gatt.humidity, humidity_percent);
}

}  // namespace co2_monitor_0601
}  // namespace esphome
#endif
