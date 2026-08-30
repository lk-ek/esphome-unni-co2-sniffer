// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "ble_options.h"
#include "history_transfer_guard.h"
#include "sensirion_bridge_core.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

#include <cstdint>
#include <string>

#if UNNI_BLE_ENABLED && !defined(USE_HOST)
#include "esphome/components/esp32_ble_server/ble_server.h"
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#endif

namespace esphome {
namespace time { class RealTimeClock; }
namespace co2_monitor_0601 {

class SensirionGadgetBridge;

class SensirionPairingModeSwitch : public switch_::Switch {
 public:
  void set_parent(SensirionGadgetBridge *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;
  SensirionGadgetBridge *parent_{nullptr};
};

class SensirionGadgetBridge : public Component {
 public:
  void setup() override;
  void loop() override;
  void prepare_for_ota();

  void set_profile(SensirionProfile profile) { this->profile_ = profile; }
  void set_temperature_source(sensor::Sensor *source) { this->temperature_source_ = source; }
  void set_humidity_source(sensor::Sensor *source) { this->humidity_source_ = source; }
  void set_history_time(time::RealTimeClock *clock) { this->history_time_ = clock; }
  void set_history_transfer_guard(HistoryTransferGuard *guard) { this->history_guard_ = guard; }

  void set_ble_device_name(const std::string &name) { this->ble_device_name_ = name; }
  void set_advertising_interval(uint32_t interval_ms);
  void set_pairing_window(uint32_t window_ms) { this->pairing_window_ms_ = window_ms; }
  void set_pairing_switch(SensirionPairingModeSwitch *value) { this->pairing_switch_ = value; }
  void set_pairing_mode(bool enabled);
  bool pairing_mode() const { return this->pairing_mode_; }

#if UNNI_BLE_ENABLED && !defined(USE_HOST)
  void set_gatt_server(esp32_ble_server::BLEServer *server) { this->gatt_server_ = server; }
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param);
#endif

  bool publish_temperature_humidity(float temperature_c, float humidity_percent);
  bool publish_co2(uint16_t ppm);
  void publish_external_temperature_humidity(float temperature_c, float humidity_percent) {
    this->publish_temperature_humidity(temperature_c, humidity_percent);
  }

  bool settings_ha_disabled() const;
  void set_settings_ha_disabled(bool disabled);

 protected:
  void setup_sources_();
  void sample_updated_();
  void process_pairing_window_();

  SensirionProfile profile_{SensirionProfile::TRH_CO2};
  sensor::Sensor *temperature_source_{nullptr};
  sensor::Sensor *humidity_source_{nullptr};
  time::RealTimeClock *history_time_{nullptr};
  HistoryTransferGuard *history_guard_{nullptr};
  std::string ble_device_name_{"Unni CO2 Monitor"};
  uint32_t advertising_interval_ms_{2000};
  uint32_t pairing_window_ms_{60000};
  uint32_t pairing_started_ms_{0};
  bool pairing_mode_{false};
  SensirionPairingModeSwitch *pairing_switch_{nullptr};
#if UNNI_BLE_ENABLED && !defined(USE_HOST)
  esp32_ble_server::BLEServer *gatt_server_{nullptr};
  bool peer_connected_{false};
  esp_bd_addr_t peer_bda_{};
#endif
};

}  // namespace co2_monitor_0601
}  // namespace esphome
