// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_monitor_0601.h"

#include "calibration.h"
#include "co2_decoder.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace esphome {
namespace co2_monitor_0601 {

static const char *const TAG = "co2_monitor_0601.host";


namespace {

bool nearly_equal(float actual, float expected, float tolerance) {
  return std::isfinite(actual) && std::isfinite(expected) && std::fabs(actual - expected) <= tolerance;
}

std::vector<std::string> split_csv_line(const std::string &line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

float parse_float(const std::string &value) {
  if (value == "nan") return NAN;
  return std::stof(value);
}

}  // namespace


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


bool CO2Monitor0601::run_capture_regression_tests_() {
  // Archived RT/RH timing captures from the 2026-08-11 reverse-engineering
  // sessions. These are intentionally kept as measured timing summaries rather
  // than hand-picked synthetic ratios. The golden engineering-unit columns were
  // frozen when this test was introduced, so later calibration changes require
  // an explicit fixture update instead of silently changing behaviour.
  const char *fixture_path = "tests/host/fixtures/rtrh_legacy_260811.csv";
  std::ifstream input(fixture_path);
  if (!input.is_open()) {
    ESP_LOGE(TAG, "capture regression: cannot open %s", fixture_path);
    return false;
  }

  std::string line;
  if (!std::getline(input, line)) {
    ESP_LOGE(TAG, "capture regression: empty fixture file");
    return false;
  }

  size_t tested = 0;
  size_t air_supported = 0;
  size_t line_no = 1;
  while (std::getline(input, line)) {
    line_no++;
    if (line.empty()) continue;
    const auto f = split_csv_line(line);
    if (f.size() != 17) {
      ESP_LOGE(TAG, "capture regression: malformed line %u (%u fields)",
               static_cast<unsigned>(line_no), static_cast<unsigned>(f.size()));
      return false;
    }

    try {
      const std::string &source = f[0];
      const int ref_count = std::stoi(f[1]);
      const float ref_period = parse_float(f[2]);
      const float ref_duration = parse_float(f[3]);
      const int rt_count = std::stoi(f[4]);
      const float rt_period = parse_float(f[5]);
      const float rt_duration = parse_float(f[6]);
      const int rh_count = std::stoi(f[7]);
      const float rh_period = parse_float(f[8]);
      const float rh_duration = parse_float(f[9]);
      const float expected_rt_ratio = parse_float(f[10]);
      const float expected_rh_ratio = parse_float(f[11]);
      const float expected_temperature = parse_float(f[12]);
      const float expected_display_temperature = parse_float(f[13]);
      const float expected_air_temperature = parse_float(f[14]);
      const float expected_humidity = parse_float(f[15]);
      const float expected_display_humidity = parse_float(f[16]);

      // Preserve the raw timing relationship from the archived capture. This
      // catches accidental fixture corruption and documents that these values
      // really originate from measured periods/counts rather than fabricated
      // calibration points.
      if (ref_count <= 0 || rt_count <= 0 || rh_count <= 0 ||
          std::fabs(ref_period * ref_count / 1000.0f - ref_duration) > 0.003f ||
          std::fabs(rt_period * rt_count / 1000.0f - rt_duration) > 0.003f ||
          std::fabs(rh_period * rh_count / 1000.0f - rh_duration) > 0.003f) {
        ESP_LOGE(TAG, "capture regression: timing/count mismatch in %s", source.c_str());
        return false;
      }

      const float rt_ratio = rt_period / ref_period;
      const float rh_ratio = rh_period / ref_period;
      if (!nearly_equal(rt_ratio, expected_rt_ratio, 0.00002f) ||
          !nearly_equal(rh_ratio, expected_rh_ratio, 0.00002f)) {
        ESP_LOGE(TAG, "capture regression: normalized timing ratio changed for %s", source.c_str());
        return false;
      }

      // Run every archived timing point through the *production* calibration
      // functions. This gives us broad regression coverage over real observed
      // RT/RH ratios rather than one representative synthetic point.
      const float temperature = calibration::temperature_from_ratio(rt_ratio);
      const float display_temperature = calibration::display_temperature_from_ratio(rt_ratio);
      const float humidity = calibration::humidity_from_ratio_temperature(rh_ratio, temperature);
      const float display_humidity =
          calibration::display_humidity_from_ratio_temperature(rh_ratio, display_temperature);
      if (!nearly_equal(temperature, expected_temperature, 0.002f) ||
          !nearly_equal(display_temperature, expected_display_temperature, 0.002f) ||
          !nearly_equal(humidity, expected_humidity, 0.003f) ||
          !nearly_equal(display_humidity, expected_display_humidity, 0.003f)) {
        ESP_LOGE(TAG, "capture regression: calibration output changed for %s", source.c_str());
        return false;
      }

      const float air_temperature = calibration::air_temperature_from_ratio(rt_ratio);
      if (std::isfinite(expected_air_temperature)) {
        air_supported++;
        if (!nearly_equal(air_temperature, expected_air_temperature, 0.002f)) {
          ESP_LOGE(TAG, "capture regression: air-temperature output changed for %s", source.c_str());
          return false;
        }
      } else if (std::isfinite(air_temperature)) {
        ESP_LOGE(TAG, "capture regression: air-temperature envelope changed for %s", source.c_str());
        return false;
      }

      tested++;
    } catch (const std::exception &err) {
      ESP_LOGE(TAG, "capture regression: parse error on line %u: %s",
               static_cast<unsigned>(line_no), err.what());
      return false;
    }
  }

  if (tested < 100) {
    ESP_LOGE(TAG, "capture regression: only %u archived captures tested", static_cast<unsigned>(tested));
    return false;
  }

  ESP_LOGI(TAG, "capture regression: %u archived RT/RH captures passed (%u within air-temperature envelope)",
           static_cast<unsigned>(tested), static_cast<unsigned>(air_supported));
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

  if (!this->run_portable_self_test_() || !this->run_capture_regression_tests_()) {
    this->mark_failed();
    return;
  }

  this->publish_fixture_values_();
  ESP_LOGI(TAG, "UNNI HOST SELF-TEST PASSED");
  // The native host process is long-lived. Emit an explicitly flushed marker
  // so automated test runners do not depend on logger/stdout buffering.
  std::fprintf(stderr, "UNNI HOST SELF-TEST PASSED\n");
  std::fflush(stderr);
}

}  // namespace co2_monitor_0601
}  // namespace esphome
