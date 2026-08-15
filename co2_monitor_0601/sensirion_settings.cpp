// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// UUID topology follows Sensirion's SHT43 DemoBoard Device Settings service.
// This compatibility probe intentionally keeps ESPHome's characteristic
// permissions and gates pairing at link/security-event level; see README.
#include "ble_options.h"
#if UNNI_BLE_ENABLED
#include "sensirion_settings.h"

#include "esphome/components/esp32_ble/ble.h"
#include "esphome/core/log.h"
#include <string>
#include <vector>

namespace esphome {
namespace co2_monitor_0601 {
namespace {
static const char *TAG = "sensirion_settings";
using esp32_ble::ESPBTUUID;
using esp32_ble_server::BLECharacteristic;

struct SettingsGatt {
  bool bound{false};
  BLECharacteristic *version{nullptr};
  BLECharacteristic *log_enabled{nullptr};
  BLECharacteristic *advertise_data_enabled{nullptr};
  BLECharacteristic *alternative_name{nullptr};
} gatt;

BLECharacteristic *get_or_create(esp32_ble_server::BLEService *service,
                                 const char *uuid, uint32_t properties) {
  const auto id = ESPBTUUID::from_raw(uuid);
  auto *characteristic = service->get_characteristic(id);
  return characteristic != nullptr ? characteristic : service->create_characteristic(id, properties);
}

void set_bool(BLECharacteristic *characteristic, bool value) {
  if (characteristic != nullptr) characteristic->set_value({static_cast<uint8_t>(value ? 1 : 0)});
}
}  // namespace

void sensirion_settings_configure_gatt(esp32_ble_server::BLEServer *server) {
  if (gatt.bound || server == nullptr) return;

  const auto service_uuid = ESPBTUUID::from_raw("00008100-B38D-4985-720E-0F993A68EE41");
  auto *service = server->get_service(service_uuid);
  if (service == nullptr) service = server->create_service(service_uuid, false, 4);
  if (service == nullptr) {
    ESP_LOGE(TAG, "failed to create Sensirion Device Settings service");
    return;
  }

  gatt.version = get_or_create(service, "000081FF-B38D-4985-720E-0F993A68EE41",
                               BLECharacteristic::PROPERTY_READ);
  gatt.log_enabled = get_or_create(service, "000081FE-B38D-4985-720E-0F993A68EE41",
                                   BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  gatt.advertise_data_enabled = get_or_create(service, "00008130-B38D-4985-720E-0F993A68EE41",
                                              BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  gatt.alternative_name = get_or_create(service, "00008120-B38D-4985-720E-0F993A68EE41",
                                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  if (gatt.version == nullptr || gatt.log_enabled == nullptr ||
      gatt.advertise_data_enabled == nullptr || gatt.alternative_name == nullptr) {
    ESP_LOGE(TAG, "failed to create complete Sensirion Device Settings topology");
    return;
  }

  gatt.version->set_value({0x01});
  set_bool(gatt.log_enabled, false);
  set_bool(gatt.advertise_data_enabled, true);
  gatt.alternative_name->set_value(std::string("MyCO2 Gadget"));

  gatt.log_enabled->on_write([](std::span<const uint8_t> x, uint16_t conn_id) {
    if (x.empty()) return;
    const bool enabled = x[0] != 0;
    set_bool(gatt.log_enabled, enabled);
    ESP_LOGW(TAG, "MyAmbience IsLogEnabled WRITE=%u (conn=%u; probe only)",
             static_cast<unsigned>(enabled), static_cast<unsigned>(conn_id));
  });
  gatt.advertise_data_enabled->on_write([](std::span<const uint8_t> x, uint16_t conn_id) {
    if (x.empty()) return;
    const bool enabled = x[0] != 0;
    set_bool(gatt.advertise_data_enabled, enabled);
    ESP_LOGW(TAG, "MyAmbience IsAdvertiseDataEnabled WRITE=%u (conn=%u; no runtime action yet)",
             static_cast<unsigned>(enabled), static_cast<unsigned>(conn_id));
  });
  gatt.alternative_name->on_write([](std::span<const uint8_t> x, uint16_t conn_id) {
    const std::string name(reinterpret_cast<const char *>(x.data()), x.size());
    gatt.alternative_name->set_value(name);
    ESP_LOGW(TAG, "MyAmbience AlternativeDeviceName WRITE len=%u (conn=%u; probe only)",
             static_cast<unsigned>(x.size()), static_cast<unsigned>(conn_id));
  });

  server->enqueue_start_service(service);
  gatt.bound = true;
  ESP_LOGI(TAG, "Sensirion Device Settings probe configured (0x8100)");
}

void sensirion_settings_gatts_event_handler(esp_gatts_cb_event_t event,
                                            esp_gatt_if_t,
                                            esp_ble_gatts_cb_param_t *param) {
  if (param == nullptr) return;
  if (event == ESP_GATTS_WRITE_EVT && !param->write.is_prep) {
    ESP_LOGD(TAG, "Device Settings GATT write handle=0x%04X len=%u conn=%u",
             static_cast<unsigned>(param->write.handle), static_cast<unsigned>(param->write.len),
             static_cast<unsigned>(param->write.conn_id));
  }
}

}  // namespace co2_monitor_0601
}  // namespace esphome
#endif
