// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "sensirion_sample.h"

#include <array>
#include <cstdint>

namespace esphome::co2_monitor_0601 {

enum class SensirionProfile : uint8_t { TRH_CO2, SHT43_TRH };

// Portable, authoritative sample state shared by live BLE, SHT43 GATT and
// persistent history. The class deliberately has no ESPHome or ESP-IDF
// dependencies so profile and coalescing behavior remains host-testable.
class SensirionBridgeCore {
 public:
  static constexpr uint32_t EXTERNAL_COALESCE_MS = 100;

  void set_profile(SensirionProfile profile);
  SensirionProfile profile() const { return this->profile_; }
  const SensirionSample &sample() const { return this->sample_; }
  bool sample_complete() const;

  bool publish_temperature_humidity(float temperature_c, float humidity_percent);
  bool publish_co2(uint16_t ppm);

  bool note_external_temperature(float temperature_c, uint32_t now_ms);
  bool note_external_humidity(float humidity_percent, uint32_t now_ms);
  bool external_update_pending() const;
  uint32_t external_due_ms() const { return this->external_due_ms_; }
  bool commit_external_if_due(uint32_t now_ms);

  std::array<uint8_t, 10> sht43_manufacturer_payload(uint16_t device_id) const;

  static bool valid_temperature(float value);
  static bool valid_humidity(float value);

 private:
  void restart_external_coalescer_(uint32_t now_ms);
  void discard_external_pending_();

  SensirionProfile profile_{SensirionProfile::TRH_CO2};
  SensirionSample sample_{};
  float pending_temperature_c_{0.0f};
  float pending_humidity_percent_{0.0f};
  uint32_t external_due_ms_{0};
  bool pending_temperature_{false};
  bool pending_humidity_{false};
};

SensirionBridgeCore &sensirion_bridge_core();

}  // namespace esphome::co2_monitor_0601
