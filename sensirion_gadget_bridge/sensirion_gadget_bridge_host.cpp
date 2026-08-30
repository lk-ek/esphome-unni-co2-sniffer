// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sensirion_gadget_bridge.h"
#include "esphome/core/log.h"

#include <array>
#include <cmath>

#ifdef USE_HOST

namespace esphome::co2_monitor_0601 {

void SensirionPairingModeSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_pairing_mode(state);
  else
    this->publish_state(state);
}

void SensirionGadgetBridge::setup() {
  SensirionBridgeCore trh;
  trh.set_profile(SensirionProfile::SHT43_TRH);
  bool passed = trh.publish_temperature_humidity(25.0f, 50.0f) && trh.sample_complete() &&
                !trh.sample().have_co2;
  const std::array<uint8_t, 10> golden{{0xD5, 0x06, 0x00, 0x06, 0x34, 0x12,
                                        0x66, 0x66, 0xB0, 0x72}};
  passed = passed && trh.sht43_manufacturer_payload(0x1234) == golden;
  passed = passed && !trh.publish_temperature_humidity(NAN, 60.0f) &&
           trh.sample().temperature_c == 25.0f && trh.sample().humidity_percent == 50.0f;

  SensirionBridgeCore co2;
  co2.set_profile(SensirionProfile::TRH_CO2);
  passed = passed && co2.publish_temperature_humidity(21.0f, 40.0f) && !co2.sample_complete();
  co2.note_external_temperature(22.0f, 1000);
  co2.note_external_humidity(42.0f, 1010);
  co2.note_external_temperature(23.0f, 1050);
  passed = passed && !co2.commit_external_if_due(1149) && co2.commit_external_if_due(1150) &&
           co2.sample().temperature_c == 23.0f && co2.sample().humidity_percent == 42.0f;

  if (!passed) {
    ESP_LOGE("sensirion_bridge.host", "SENSIRION BRIDGE HOST SELF-TEST FAILED");
    this->mark_failed();
    return;
  }
  sensirion_bridge_core().set_profile(this->profile_);
  this->setup_sources_();
  if (this->pairing_switch_ != nullptr) this->pairing_switch_->publish_state(false);
  ESP_LOGI("sensirion_bridge.host", "SENSIRION BRIDGE HOST SELF-TEST PASSED");
}

void SensirionGadgetBridge::loop() { this->process_pairing_window_(); }
void SensirionGadgetBridge::prepare_for_ota() {}

void SensirionGadgetBridge::set_advertising_interval(uint32_t interval_ms) {
  this->advertising_interval_ms_ = interval_ms;
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
}

void SensirionGadgetBridge::sample_updated_() {}

bool SensirionGadgetBridge::publish_temperature_humidity(float temperature_c, float humidity_percent) {
  return sensirion_bridge_core().publish_temperature_humidity(temperature_c, humidity_percent);
}

bool SensirionGadgetBridge::publish_co2(uint16_t ppm) {
  return sensirion_bridge_core().publish_co2(ppm);
}

void SensirionGadgetBridge::set_pairing_mode(bool enabled) {
  this->pairing_mode_ = enabled;
  this->pairing_started_ms_ = enabled ? millis() : 0;
  if (this->pairing_switch_ != nullptr) this->pairing_switch_->publish_state(enabled);
}

void SensirionGadgetBridge::process_pairing_window_() {
  if (this->pairing_mode_ && static_cast<uint32_t>(millis() - this->pairing_started_ms_) >= this->pairing_window_ms_)
    this->set_pairing_mode(false);
}

bool SensirionGadgetBridge::settings_ha_disabled() const { return false; }
void SensirionGadgetBridge::set_settings_ha_disabled(bool) {}

}  // namespace esphome::co2_monitor_0601

#endif
