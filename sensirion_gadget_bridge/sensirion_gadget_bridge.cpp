// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sensirion_gadget_bridge.h"

#include "sensirion_ble.h"
#include "sensirion_history.h"
#include "sensirion_settings.h"
#include "sensirion_sht43_probe.h"

#include "esphome/core/log.h"

#if UNNI_BLE_ENABLED
#include "esphome/components/esp32_ble/ble.h"
#include "esp_err.h"
#include <cstring>
#endif

namespace esphome::co2_monitor_0601 {
namespace {
static const char *const TAG = "sensirion_bridge";
}

void SensirionPairingModeSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_pairing_mode(state);
  else
    this->publish_state(state);
}

void SensirionGadgetBridge::set_advertising_interval(uint32_t interval_ms) {
  this->advertising_interval_ms_ = interval_ms;
#if UNNI_BLE_ENABLED
  sensirion_ble_set_advertising_interval(interval_ms);
#endif
}

void SensirionGadgetBridge::setup_sources_() {
  if (this->temperature_source_ == nullptr || this->humidity_source_ == nullptr) return;
  this->temperature_source_->add_on_state_callback([this](float value) {
    if (!sensirion_bridge_core().note_external_temperature(value, millis())) return;
    this->set_timeout("sensirion-trh-coalesce", SensirionBridgeCore::EXTERNAL_COALESCE_MS, [this]() {
      if (sensirion_bridge_core().commit_external_if_due(millis())) this->sample_updated_();
    });
  });
  this->humidity_source_->add_on_state_callback([this](float value) {
    if (!sensirion_bridge_core().note_external_humidity(value, millis())) return;
    this->set_timeout("sensirion-trh-coalesce", SensirionBridgeCore::EXTERNAL_COALESCE_MS, [this]() {
      if (sensirion_bridge_core().commit_external_if_due(millis())) this->sample_updated_();
    });
  });
  ESP_LOGI(TAG, "external T/RH sources registered with %u ms coalescing",
           static_cast<unsigned>(SensirionBridgeCore::EXTERNAL_COALESCE_MS));
}

void SensirionGadgetBridge::sample_updated_() {
#if UNNI_BLE_ENABLED
  const auto &sample = sensirion_bridge_core().sample();
  if (sample.temperature_humidity_complete())
    sensirion_ble_set_temperature_humidity(sample.temperature_c, sample.humidity_percent);
#if UNNI_BLE_LIVE_ENABLED
  sensirion_ble_commit_live_advertisement();
#endif
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_on_sample_updated();
#endif
}

bool SensirionGadgetBridge::publish_temperature_humidity(float temperature_c,
                                                         float humidity_percent) {
#if UNNI_BLE_ENABLED
  sensirion_ble_set_temperature_humidity(temperature_c, humidity_percent);
  if (!sensirion_bridge_core().sample().temperature_humidity_complete()) return false;
#else
  if (!sensirion_bridge_core().publish_temperature_humidity(temperature_c, humidity_percent)) return false;
#endif
  this->sample_updated_();
  return true;
}

bool SensirionGadgetBridge::publish_co2(uint16_t ppm) {
#if UNNI_BLE_ENABLED
  sensirion_ble_set_co2(ppm);
#else
  if (!sensirion_bridge_core().publish_co2(ppm)) return false;
#endif
  this->sample_updated_();
  return true;
}

void SensirionGadgetBridge::set_pairing_mode(bool enabled) {
  this->pairing_mode_ = enabled;
  this->pairing_started_ms_ = enabled ? millis() : 0;
  if (this->pairing_switch_ != nullptr) this->pairing_switch_->publish_state(enabled);
#if UNNI_BLE_ENABLED
  if (enabled && this->peer_connected_) {
    const esp_err_t err = esp_ble_set_encryption(this->peer_bda_, ESP_BLE_SEC_ENCRYPT_MITM);
    if (err != ESP_OK) ESP_LOGW(TAG, "pairing encryption request failed: %s", esp_err_to_name(err));
  }
#endif
  ESP_LOGW(TAG, "BLE pairing window %s", enabled ? "opened" : "closed");
}

void SensirionGadgetBridge::process_pairing_window_() {
  if (!this->pairing_mode_) return;
  if (static_cast<uint32_t>(millis() - this->pairing_started_ms_) < this->pairing_window_ms_) return;
  this->set_pairing_mode(false);
}

bool SensirionGadgetBridge::settings_ha_disabled() const {
#if UNNI_BLE_ENABLED
  return sensirion_settings_ha_disabled();
#else
  return false;
#endif
}

void SensirionGadgetBridge::set_settings_ha_disabled(bool disabled) {
#if UNNI_BLE_ENABLED
  sensirion_settings_set_ha_disabled(disabled);
#else
  (void) disabled;
#endif
}

void SensirionGadgetBridge::prepare_for_ota() {
#if UNNI_BLE_HISTORY_ENABLED
  if (!sensirion_history_flush()) ESP_LOGW(TAG, "OTA: history flush failed");
#endif
#if UNNI_BLE_ENABLED
  if (!sensirion_settings_flush()) ESP_LOGW(TAG, "OTA: settings flush failed");
#endif
}

void SensirionGadgetBridge::setup() {
  sensirion_bridge_core().set_profile(this->profile_);
  this->setup_sources_();
  if (this->pairing_switch_ != nullptr) this->pairing_switch_->publish_state(false);

#if UNNI_BLE_ENABLED
  sensirion_ble_setup();
  sensirion_ble_set_advertising_interval(this->advertising_interval_ms_);
  if (this->gatt_server_ == nullptr) {
    ESP_LOGE(TAG, "BLE enabled but no GATT server is available");
  } else {
    auto *info = this->gatt_server_->get_service(esp32_ble::ESPBTUUID::from_uint16(0x180A));
    if (info != nullptr) {
#if UNNI_SHT43_IDENTITY_PROBE
      if (auto *value = info->get_characteristic(0x2A29)) value->set_value(std::string("Sensirion"));
      if (auto *value = info->get_characteristic(0x2A24)) value->set_value(std::string("SHT43 DB"));
      if (auto *value = info->get_characteristic(0x2A26)) value->set_value(std::string("1.0.0"));
#else
      if (auto *value = info->get_characteristic(0x2A29)) value->set_value(std::string("Gadget"));
      if (auto *value = info->get_characteristic(0x2A24)) value->set_value(this->ble_device_name_);
      if (auto *value = info->get_characteristic(0x2A26)) value->set_value(std::string("1.0.1"));
#endif
    }
#if UNNI_BLE_HISTORY_ENABLED
    sensirion_history_configure_gatt(this->gatt_server_);
#endif
#if UNNI_SHT43_IDENTITY_PROBE
    sensirion_sht43_probe_configure_gatt(this->gatt_server_);
#endif
    sensirion_settings_configure_gatt(this->gatt_server_, this->ble_device_name_);
  }
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_setup(this->history_time_);
#endif
}

void SensirionGadgetBridge::loop() {
#if UNNI_BLE_ENABLED
  sensirion_ble_loop();
  sensirion_settings_loop();
  this->process_pairing_window_();
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_loop(this->history_guard_);
#endif
}

#if UNNI_BLE_ENABLED
void SensirionGadgetBridge::gap_event_handler(esp_gap_ble_cb_event_t event,
                                              esp_ble_gap_cb_param_t *param) {
  sensirion_ble_gap_event_handler(event, param);
  if (param == nullptr) return;
  switch (event) {
    case ESP_GAP_BLE_SEC_REQ_EVT:
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    case ESP_GAP_BLE_NC_REQ_EVT:
      esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, this->pairing_mode_);
      ESP_LOGW(TAG, "numeric comparison %06lu: %s",
               static_cast<unsigned long>(param->ble_security.key_notif.passkey),
               this->pairing_mode_ ? "accepted" : "rejected");
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      if (param->ble_security.auth_cmpl.success && this->pairing_mode_) this->set_pairing_mode(false);
      break;
    default:
      break;
  }
}

void SensirionGadgetBridge::gatts_event_handler(esp_gatts_cb_event_t event,
                                                esp_gatt_if_t gatts_if,
                                                esp_ble_gatts_cb_param_t *param) {
  sensirion_ble_gatts_event_handler(event, param);
  sensirion_settings_gatts_event_handler(event, gatts_if, param);
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_gatts_event_handler(event, gatts_if, param);
#endif
  if (event == ESP_GATTS_CONNECT_EVT && param != nullptr) {
    this->peer_connected_ = true;
    std::memcpy(this->peer_bda_, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    if (this->pairing_mode_)
      esp_ble_set_encryption(this->peer_bda_, ESP_BLE_SEC_ENCRYPT_MITM);
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    this->peer_connected_ = false;
    std::memset(this->peer_bda_, 0, sizeof(esp_bd_addr_t));
  }
}
#endif

}  // namespace esphome::co2_monitor_0601
