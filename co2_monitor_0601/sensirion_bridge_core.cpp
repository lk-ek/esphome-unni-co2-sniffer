// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sensirion_bridge_core.h"

#include <cmath>

namespace esphome::co2_monitor_0601 {
namespace {
constexpr uint8_t COMPANY_ID_LO = 0xD5;
constexpr uint8_t COMPANY_ID_HI = 0x06;
constexpr uint8_t ADV_TYPE_SAMPLE = 0x00;
constexpr uint8_t SAMPLE_TYPE_SHT43 = 0x06;

uint16_t sht43_ticks(float value) {
  if (value <= 0.0f) return 0;
  if (value >= 65535.0f) return 65535;
  return static_cast<uint16_t>(value + 0.5f);
}
}  // namespace

void SensirionBridgeCore::set_profile(SensirionProfile profile) {
  this->profile_ = profile;
  if (profile == SensirionProfile::SHT43_TRH) {
    this->sample_.co2_ppm = 0;
    this->sample_.have_co2 = false;
  }
}

bool SensirionBridgeCore::sample_complete() const {
  return this->profile_ == SensirionProfile::SHT43_TRH
             ? this->sample_.temperature_humidity_complete()
             : this->sample_.complete();
}

bool SensirionBridgeCore::valid_temperature(float value) {
  return std::isfinite(value) && value >= -45.0f && value <= 130.0f;
}

bool SensirionBridgeCore::valid_humidity(float value) {
  return std::isfinite(value) && value >= 0.0f && value <= 100.0f;
}

bool SensirionBridgeCore::publish_temperature_humidity(float temperature_c,
                                                       float humidity_percent) {
  if (!valid_temperature(temperature_c) || !valid_humidity(humidity_percent)) return false;
  this->sample_.temperature_c = temperature_c;
  this->sample_.humidity_percent = humidity_percent;
  this->sample_.have_temperature = true;
  this->sample_.have_humidity = true;
  this->discard_external_pending_();
  return true;
}

bool SensirionBridgeCore::publish_co2(uint16_t ppm) {
  if (this->profile_ == SensirionProfile::SHT43_TRH) return false;
  this->sample_.co2_ppm = ppm;
  this->sample_.have_co2 = true;
  return true;
}

void SensirionBridgeCore::restart_external_coalescer_(uint32_t now_ms) {
  this->external_due_ms_ = now_ms + EXTERNAL_COALESCE_MS;
}

void SensirionBridgeCore::discard_external_pending_() {
  this->pending_temperature_ = false;
  this->pending_humidity_ = false;
}

bool SensirionBridgeCore::note_external_temperature(float temperature_c, uint32_t now_ms) {
  if (!valid_temperature(temperature_c)) {
    this->discard_external_pending_();
    return false;
  }
  this->pending_temperature_c_ = temperature_c;
  this->pending_temperature_ = true;
  this->restart_external_coalescer_(now_ms);
  return true;
}

bool SensirionBridgeCore::note_external_humidity(float humidity_percent, uint32_t now_ms) {
  if (!valid_humidity(humidity_percent)) {
    this->discard_external_pending_();
    return false;
  }
  this->pending_humidity_percent_ = humidity_percent;
  this->pending_humidity_ = true;
  this->restart_external_coalescer_(now_ms);
  return true;
}

bool SensirionBridgeCore::external_update_pending() const {
  return this->pending_temperature_ && this->pending_humidity_;
}

bool SensirionBridgeCore::commit_external_if_due(uint32_t now_ms) {
  if (static_cast<int32_t>(now_ms - this->external_due_ms_) < 0) return false;
  if (!this->external_update_pending()) {
    this->discard_external_pending_();
    return false;
  }
  const bool committed = this->publish_temperature_humidity(
      this->pending_temperature_c_, this->pending_humidity_percent_);
  return committed;
}

std::array<uint8_t, 10> SensirionBridgeCore::sht43_manufacturer_payload(
    uint16_t device_id) const {
  std::array<uint8_t, 10> payload{};
  if (this->profile_ != SensirionProfile::SHT43_TRH || !this->sample_complete()) return payload;
  const uint16_t t_ticks = sht43_ticks((this->sample_.temperature_c + 45.0f) * 65535.0f / 175.0f);
  const uint16_t rh_ticks = sht43_ticks((this->sample_.humidity_percent + 6.0f) * 65535.0f / 125.0f);
  payload = {COMPANY_ID_LO, COMPANY_ID_HI, ADV_TYPE_SAMPLE, SAMPLE_TYPE_SHT43,
             static_cast<uint8_t>(device_id & 0xFF), static_cast<uint8_t>(device_id >> 8),
             static_cast<uint8_t>(t_ticks & 0xFF), static_cast<uint8_t>(t_ticks >> 8),
             static_cast<uint8_t>(rh_ticks & 0xFF), static_cast<uint8_t>(rh_ticks >> 8)};
  return payload;
}

SensirionBridgeCore &sensirion_bridge_core() {
  static SensirionBridgeCore core;
  return core;
}

}  // namespace esphome::co2_monitor_0601
