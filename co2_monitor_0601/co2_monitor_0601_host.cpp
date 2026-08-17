// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_monitor_0601.h"

#include "calibration.h"
#include "co2_decoder.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome {
namespace co2_monitor_0601 {

static const char *const TAG = "co2_monitor_0601.host";

void EnergySaveModeSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) this->parent_->set_energy_save_mode(state);
  this->publish_state(state);
}

void BlePairingModeSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) this->parent_->set_ble_pairing_mode(state);
  this->publish_state(state);
}

void WifiHaSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) this->parent_->set_wifi_ha_enabled(state);
  this->publish_state(state);
}

bool CO2Monitor0601::run_portable_self_test_() {
  using namespace i2c_sniffer;

  // Known-good EC05 command + 500 ppm response. CRC(0x01, 0xF4) = 0x33.
  Capture capture{};
  capture.frame_count = 2;
  auto &command = capture.frames[0];
  command.address = 0x62;
  command.direction = Direction::Write;
  command.address_ack = true;
  command.length = 2;
  command.data[0] = 0xEC;
  command.data[1] = 0x05;
  command.ack[0] = true;
  command.ack[1] = true;
  command.end_condition = EndCondition::Stop;

  auto &response = capture.frames[1];
  response.address = 0x62;
  response.direction = Direction::Read;
  response.address_ack = true;
  response.length = 3;
  response.data[0] = 0x01;
  response.data[1] = 0xF4;
  response.data[2] = 0x33;
  response.ack[0] = true;
  response.ack[1] = true;
  response.ack[2] = false;
  response.end_condition = EndCondition::Stop;

  if (!co2_decoder::validate_measurement_capture(capture)) {
    ESP_LOGE(TAG, "portable self-test: valid EC05 capture rejected");
    return false;
  }

  co2_decoder::Result result{};
  if (!co2_decoder::process_frame(response, result) || !result.have_co2 || result.co2_ppm != 500) {
    ESP_LOGE(TAG, "portable self-test: 500 ppm response decoded incorrectly");
    return false;
  }

  // A changed CRC must be rejected by the strict capture validator and counted
  // by the frame consumer without publishing a CO2 value.
  capture.frames[1].data[2] ^= 0x01;
  if (co2_decoder::validate_measurement_capture(capture)) {
    ESP_LOGE(TAG, "portable self-test: invalid CRC accepted");
    return false;
  }
  co2_decoder::Result bad_crc{};
  co2_decoder::process_frame(capture.frames[1], bad_crc);
  if (bad_crc.crc_errors != 1 || bad_crc.have_co2) {
    ESP_LOGE(TAG, "portable self-test: CRC error accounting failed");
    return false;
  }

  // Exercise the portable calibration path with a representative measured
  // ratio and basic invariants rather than pinning a fragile rounded value.
  constexpr float rt_ratio = 1.992783f;
  constexpr float rh_ratio = 4.056572f;
  const float temperature = calibration::temperature_from_ratio(rt_ratio);
  const float humidity = calibration::humidity_from_ratio_temperature(rh_ratio, temperature);
  const float display_temperature = calibration::display_temperature_from_ratio(rt_ratio);
  const float display_humidity = calibration::display_humidity_from_ratio_temperature(rh_ratio, display_temperature);
  if (!std::isfinite(temperature) || !std::isfinite(humidity) || humidity < 0.0f || humidity > 100.0f ||
      !std::isfinite(display_temperature) || !std::isfinite(display_humidity) || display_humidity < 0.0f ||
      display_humidity > 100.0f) {
    ESP_LOGE(TAG, "portable self-test: calibration invariant failed");
    return false;
  }

  return true;
}

void CO2Monitor0601::publish_fixture_values_() {
  constexpr float rt_ratio = 1.992783f;
  constexpr float rh_ratio = 4.056572f;
  const float temperature = calibration::temperature_from_ratio(rt_ratio);
  const float humidity = calibration::humidity_from_ratio_temperature(rh_ratio, temperature);
  const float display_temperature = calibration::display_temperature_from_ratio(rt_ratio);
  const float display_humidity = calibration::display_humidity_from_ratio_temperature(rh_ratio, display_temperature);
  const float air_temperature = calibration::air_temperature_from_ratio(rt_ratio);

  if (this->co2_ != nullptr) this->co2_->publish_state(500.0f);
  if (this->crc_errors_ != nullptr) this->crc_errors_->publish_state(0.0f);
  if (this->frame_errors_ != nullptr) this->frame_errors_->publish_state(0.0f);
  if (this->rt_temperature_ != nullptr) this->rt_temperature_->publish_state(temperature);
  if (this->air_temperature_ != nullptr && std::isfinite(air_temperature)) this->air_temperature_->publish_state(air_temperature);
  if (this->display_temperature_ != nullptr) this->display_temperature_->publish_state(display_temperature);
  if (this->rh_humidity_ != nullptr) this->rh_humidity_->publish_state(humidity);
  if (this->display_humidity_ != nullptr) this->display_humidity_->publish_state(display_humidity);
  if (this->battery_voltage_ != nullptr) this->battery_voltage_->publish_state(3.900f);
  if (this->battery_level_ != nullptr) this->battery_level_->publish_state(50.0f);
  if (this->battery_runtime_estimate_ != nullptr) this->battery_runtime_estimate_->publish_state(12.0f);
  if (this->battery_charge_time_estimate_ != nullptr) this->battery_charge_time_estimate_->publish_state(1.5f);
  if (this->battery_discharge_rate_ != nullptr) this->battery_discharge_rate_->publish_state(4.0f);
  if (this->battery_charge_rate_ != nullptr) this->battery_charge_rate_->publish_state(20.0f);
  if (this->battery_learned_full_runtime_ != nullptr) this->battery_learned_full_runtime_->publish_state(24.0f);
  if (this->battery_learning_progress_ != nullptr) this->battery_learning_progress_->publish_state(50.0f);
  if (this->battery_learning_cycles_ != nullptr) this->battery_learning_cycles_->publish_state(1.0f);
  if (this->ref_period_ != nullptr) this->ref_period_->publish_state(77.529f);
  if (this->rt_period_ != nullptr) this->rt_period_->publish_state(154.498f);
  if (this->rh_state_period_ != nullptr) this->rh_state_period_->publish_state(336.0f);
  if (this->rt_ratio_ != nullptr) this->rt_ratio_->publish_state(rt_ratio);
  if (this->rh_ratio_ != nullptr) this->rh_ratio_->publish_state(rh_ratio);
  if (this->rh_log_ != nullptr) this->rh_log_->publish_state(calibration::log_rh_ratio(rh_ratio));
  if (this->measurement_quality_ != nullptr) this->measurement_quality_->publish_state(93.0f);
  if (this->usb_power_ != nullptr) this->usb_power_->publish_state(false);
  if (this->thermal_transient_ != nullptr) this->thermal_transient_->publish_state(false);
  if (this->temperature_extrapolation_ != nullptr) this->temperature_extrapolation_->publish_state(false);
  if (this->humidity_extrapolation_ != nullptr) this->humidity_extrapolation_->publish_state(false);
  if (this->calibration_extrapolation_ != nullptr)
    this->calibration_extrapolation_->publish_state(calibration::is_extrapolation(temperature, rh_ratio));

  if (this->energy_save_switch_ != nullptr) this->energy_save_switch_->publish_state(this->energy_save_mode_);
  if (this->ble_pairing_switch_ != nullptr) this->ble_pairing_switch_->publish_state(this->ble_pairing_mode_);
  if (this->wifi_ha_switch_ != nullptr) this->wifi_ha_switch_->publish_state(this->wifi_ha_enabled_);
}

void CO2Monitor0601::setup() {
  ESP_LOGI(TAG,
           "host smoke target: sniffer=%s rtrh=%s debug_metrics=%s light_sleep=%s BLE=%d BLE history=%d HA=%d",
           YESNO(this->sniffer_enabled_), YESNO(this->rtrh_enabled_), YESNO(this->debug_metrics_), YESNO(this->light_sleep_),
           UNNI_BLE_ENABLED, UNNI_BLE_HISTORY_ENABLED, UNNI_HOME_ASSISTANT_ENABLED);

  if (!this->run_portable_self_test_()) {
    this->mark_failed();
    return;
  }

  this->publish_fixture_values_();
  ESP_LOGI(TAG, "UNNI HOST SELF-TEST PASSED");
}

}  // namespace co2_monitor_0601
}  // namespace esphome
