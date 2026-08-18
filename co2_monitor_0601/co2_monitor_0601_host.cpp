// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_monitor_0601.h"

#include "calibration.h"
#include "battery_learning.h"
#include "power_policy_logic.h"
#include "co2_decoder.h"
#include "esphome/core/log.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <climits>
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

std::string trim_ascii_whitespace(const std::string &value) {
  size_t begin = 0;
  while (begin < value.size()) {
    const char c = value[begin];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    begin++;
  }
  size_t end = value.size();
  while (end > begin) {
    const char c = value[end - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    end--;
  }
  return value.substr(begin, end - begin);
}

bool parse_int(const std::string &value, int &out) {
  const std::string trimmed = trim_ascii_whitespace(value);
  if (trimmed.empty()) return false;
  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(trimmed.c_str(), &end, 10);
  if (errno != 0 || end == trimmed.c_str() || *end != '\0' || parsed < INT_MIN || parsed > INT_MAX) return false;
  out = static_cast<int>(parsed);
  return true;
}

bool parse_float(const std::string &value, float &out) {
  const std::string trimmed = trim_ascii_whitespace(value);
  if (trimmed == "nan") {
    out = NAN;
    return true;
  }
  if (trimmed.empty()) return false;
  errno = 0;
  char *end = nullptr;
  const float parsed = std::strtof(trimmed.c_str(), &end);
  if (errno != 0 || end == trimmed.c_str() || *end != '\0') return false;
  out = parsed;
  return true;
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


bool CO2Monitor0601::run_battery_power_policy_tests_() {
  using battery_learning::State;

  // Learning progress is governed by both elapsed time and discharged SOC.
  State learning{};
  if (!nearly_equal(battery_learning::progress_percent(learning), 0.0f, 0.001f)) {
    ESP_LOGE(TAG, "battery learning self-test: inactive progress is not zero");
    return false;
  }

  battery_learning::update(learning, 80.0f, 1000U);
  battery_learning::update(learning, 76.0f, 1000U + 3600000U);
  if (!nearly_equal(battery_learning::progress_percent(learning), 50.0f, 0.01f)) {
    ESP_LOGE(TAG, "battery learning self-test: qualification progress mismatch");
    return false;
  }
  battery_learning::update(learning, 72.0f, 1000U + 7200000U);
  if (!nearly_equal(battery_learning::progress_percent(learning), 100.0f, 0.01f)) {
    ESP_LOGE(TAG, "battery learning self-test: qualified session did not reach 100%%");
    return false;
  }
  auto finalized = battery_learning::finalize(learning, true);
  if (!finalized.model_updated || !nearly_equal(finalized.observed_full_runtime_h, 25.0f, 0.001f) ||
      !nearly_equal(learning.learned_full_runtime_h, 25.0f, 0.001f) || learning.learned_cycles != 1 ||
      learning.session_active) {
    ESP_LOGE(TAG, "battery learning self-test: first qualified session mismatch");
    return false;
  }

  // The second qualified session must update the model using the production
  // 25/75 EMA: 20 h observed -> 23.75 h learned from the previous 25 h model.
  battery_learning::update(learning, 90.0f, 10000U);
  battery_learning::update(learning, 80.0f, 10000U + 7200000U);
  finalized = battery_learning::finalize(learning, true);
  if (!finalized.model_updated || !nearly_equal(finalized.observed_full_runtime_h, 20.0f, 0.001f) ||
      !nearly_equal(learning.learned_full_runtime_h, 23.75f, 0.001f) || learning.learned_cycles != 2) {
    ESP_LOGE(TAG, "battery learning self-test: EMA update mismatch");
    return false;
  }

  // Too-short, too-small and incomplete sessions must never alter the model.
  const float learned_before_rejects = learning.learned_full_runtime_h;
  const uint16_t cycles_before_rejects = learning.learned_cycles;
  battery_learning::update(learning, 80.0f, 20000U);
  battery_learning::update(learning, 70.0f, 20000U + 3600000U);
  if (battery_learning::finalize(learning, true).model_updated ||
      !nearly_equal(learning.learned_full_runtime_h, learned_before_rejects, 0.001f)) {
    ESP_LOGE(TAG, "battery learning self-test: short session contaminated model");
    return false;
  }
  battery_learning::update(learning, 80.0f, 30000U);
  battery_learning::update(learning, 75.0f, 30000U + 3U * 3600000U);
  if (battery_learning::finalize(learning, true).model_updated || learning.learned_cycles != cycles_before_rejects) {
    ESP_LOGE(TAG, "battery learning self-test: low-drop session contaminated model");
    return false;
  }
  battery_learning::update(learning, 80.0f, 40000U);
  battery_learning::update(learning, 60.0f, 40000U + 3U * 3600000U);
  if (battery_learning::finalize(learning, false).model_updated || learning.learned_cycles != cycles_before_rejects) {
    ESP_LOGE(TAG, "battery learning self-test: incomplete session contaminated model");
    return false;
  }

  // uint32_t millis() wrap-around must preserve elapsed time accounting.
  State wrap{};
  wrap.session_active = true;
  wrap.last_tick_ms = 0xFFFFFFF0U;
  battery_learning::advance_elapsed(wrap, 0x00000010U);
  if (wrap.session_elapsed_ms != 32U || wrap.last_tick_ms != 0x00000010U) {
    ESP_LOGE(TAG, "battery learning self-test: millis wrap accounting failed");
    return false;
  }

  using namespace power_policy_logic;
  power_policy_logic::State policy{};
  if (!external_powered(true, policy) || external_powered(false, policy)) {
    ESP_LOGE(TAG, "power policy self-test: automatic USB/battery selection failed");
    return false;
  }

  auto action = set_mode(policy, true, true, 1000U);
  if (action != SetModeAction::StartGrace || !policy.grace_pending || policy.policy_active ||
      !external_powered(true, policy)) {
    ESP_LOGE(TAG, "power policy self-test: USB grace start mismatch");
    return false;
  }
  if (process_grace(policy, 3999U) || !external_powered(true, policy)) {
    ESP_LOGE(TAG, "power policy self-test: grace ended too early");
    return false;
  }
  if (!process_grace(policy, 4000U) || policy.grace_pending || !policy.policy_active ||
      external_powered(true, policy)) {
    ESP_LOGE(TAG, "power policy self-test: grace completion mismatch");
    return false;
  }

  action = set_mode(policy, false, true, 5000U);
  if (action != SetModeAction::ApplyUsbPolicy || policy.policy_active || policy.grace_pending ||
      !external_powered(true, policy)) {
    ESP_LOGE(TAG, "power policy self-test: disabling override did not restore USB policy");
    return false;
  }

  action = set_mode(policy, true, false, 6000U);
  if (action != SetModeAction::ApplyBatteryPolicy || !policy.policy_active || policy.grace_pending ||
      external_powered(false, policy)) {
    ESP_LOGE(TAG, "power policy self-test: battery enable should apply immediately");
    return false;
  }
  if (set_mode(policy, true, false, 7000U) != SetModeAction::NoChange) {
    ESP_LOGE(TAG, "power policy self-test: unchanged mode produced an action");
    return false;
  }

  power_policy_logic::State no_grace{};
  no_grace.grace_ms = 0;
  if (set_mode(no_grace, true, true, 1U) != SetModeAction::ApplyBatteryPolicy ||
      !no_grace.policy_active || external_powered(true, no_grace)) {
    ESP_LOGE(TAG, "power policy self-test: zero grace did not force battery policy immediately");
    return false;
  }

  if (ble_advertising_interval(true, 1000U, 3000U) != 1000U ||
      ble_advertising_interval(false, 1000U, 3000U) != 3000U) {
    ESP_LOGE(TAG, "power policy self-test: BLE cadence selection mismatch");
    return false;
  }

  return true;
}



bool CO2Monitor0601::run_co2_decoder_hardening_tests_() {
  using namespace i2c_sniffer;

  struct Sample {
    std::string source;
    uint16_t ppm;
    uint8_t msb;
    uint8_t lsb;
    uint8_t crc;
  };

  const char *fixture_path = "tests/host/fixtures/co2_current_260817.csv";
  std::ifstream input(fixture_path);
  if (!input.is_open()) {
    ESP_LOGE(TAG, "CO2 hardening: cannot open %s", fixture_path);
    return false;
  }

  std::string line;
  if (!std::getline(input, line)) return false;
  std::vector<Sample> samples;
  size_t line_no = 1;
  while (std::getline(input, line)) {
    line_no++;
    if (line.empty()) continue;
    const auto f = split_csv_line(line);
    if (f.size() != 5) {
      ESP_LOGE(TAG, "CO2 hardening: malformed corpus line %u", static_cast<unsigned>(line_no));
      return false;
    }
    int ppm = 0, msb = 0, lsb = 0, crc = 0;
    if (!parse_int(f[1], ppm) || !parse_int(f[2], msb) || !parse_int(f[3], lsb) || !parse_int(f[4], crc) ||
        ppm < 0 || ppm > 65535 || msb < 0 || msb > 255 || lsb < 0 || lsb > 255 || crc < 0 || crc > 255 ||
        ppm != ((msb << 8) | lsb)) {
      ESP_LOGE(TAG, "CO2 hardening: invalid corpus line %u", static_cast<unsigned>(line_no));
      return false;
    }
    samples.push_back({f[0], static_cast<uint16_t>(ppm), static_cast<uint8_t>(msb), static_cast<uint8_t>(lsb),
                       static_cast<uint8_t>(crc)});
  }
  if (samples.size() < 50) {
    ESP_LOGE(TAG, "CO2 hardening: only %u current I2C samples", static_cast<unsigned>(samples.size()));
    return false;
  }

  auto make_capture = [](const Sample &sample) {
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
    response.data[0] = sample.msb;
    response.data[1] = sample.lsb;
    response.data[2] = sample.crc;
    response.ack[0] = true;
    response.ack[1] = true;
    response.ack[2] = false;
    response.end_condition = EndCondition::Stop;
    return capture;
  };

  // Every independently recovered CRC-valid response from the current raw
  // edge-capture archive must still decode when paired with the observed EC05
  // command. This provides real-wire provenance without putting raw captures
  // into the repository.
  for (const auto &sample : samples) {
    const Capture capture = make_capture(sample);
    co2_decoder::Result result{};
    if (!co2_decoder::validate_measurement_capture(capture) ||
        !co2_decoder::process_capture(capture, result) || !result.have_co2 || result.co2_ppm != sample.ppm ||
        result.crc_errors != 0 || result.frame_errors != 0) {
      ESP_LOGE(TAG, "CO2 hardening: current capture rejected: %s", sample.source.c_str());
      return false;
    }
  }

  const Sample &base = samples.front();
  const Capture pristine = make_capture(base);
  auto expect_rejected = [&](Capture mutated, const char *label) {
    co2_decoder::Result result{};
    co2_decoder::process_capture(mutated, result);
    if (result.have_co2) {
      ESP_LOGE(TAG, "CO2 hardening: mutation accepted: %s", label);
      return false;
    }
    // A bad capture must not poison a subsequent independent capture.
    co2_decoder::Result recovered{};
    if (!co2_decoder::process_capture(pristine, recovered) || !recovered.have_co2 || recovered.co2_ppm != base.ppm) {
      ESP_LOGE(TAG, "CO2 hardening: failed to recover after mutation: %s", label);
      return false;
    }
    return true;
  };

#define UNNI_EXPECT_REJECTED(expr, label) \
  do {                                    \
    Capture c = pristine;                 \
    expr;                                 \
    if (!expect_rejected(c, label)) return false; \
  } while (0)

  UNNI_EXPECT_REJECTED(c.frames[1].data[0] ^= 0x01, "response data bit flip");
  UNNI_EXPECT_REJECTED(c.frames[1].data[1] ^= 0x80, "response data high-bit flip");
  UNNI_EXPECT_REJECTED(c.frames[1].data[2] ^= 0x01, "CRC bit flip");
  UNNI_EXPECT_REJECTED(c.frames[1].length = 2, "short response");
  UNNI_EXPECT_REJECTED(c.frames[1].length = 4, "extra response byte");
  UNNI_EXPECT_REJECTED(c.frames[1].address = 0x63, "wrong response address");
  UNNI_EXPECT_REJECTED(c.frames[1].direction = Direction::Write, "wrong response direction");
  UNNI_EXPECT_REJECTED(c.frames[1].address_ack = false, "response address NACK");
  UNNI_EXPECT_REJECTED(c.frames[1].ack[0] = false, "response first data NACK");
  UNNI_EXPECT_REJECTED(c.frames[1].ack[1] = false, "response second data NACK");
  UNNI_EXPECT_REJECTED(c.frames[1].ack[2] = true, "response final ACK");
  UNNI_EXPECT_REJECTED(c.frames[1].end_condition = EndCondition::CaptureEnd, "response without STOP");
  UNNI_EXPECT_REJECTED(c.frames[0].data[0] = 0xED, "wrong command high byte");
  UNNI_EXPECT_REJECTED(c.frames[0].data[1] = 0x04, "wrong command low byte");
  UNNI_EXPECT_REJECTED(c.frames[0].length = 1, "short command");
  UNNI_EXPECT_REJECTED(c.frames[0].length = 3, "extra command byte");
  UNNI_EXPECT_REJECTED(c.frames[0].address = 0x63, "wrong command address");
  UNNI_EXPECT_REJECTED(c.frames[0].direction = Direction::Read, "wrong command direction");
  UNNI_EXPECT_REJECTED(c.frames[0].address_ack = false, "command address NACK");
  UNNI_EXPECT_REJECTED(c.frames[0].ack[0] = false, "command first data NACK");
  UNNI_EXPECT_REJECTED(c.frames[0].ack[1] = false, "command second data NACK");
  UNNI_EXPECT_REJECTED(c.frames[0].end_condition = EndCondition::RepeatedStart, "command repeated START");
  UNNI_EXPECT_REJECTED(c.frame_count = 1; c.frames[0] = c.frames[1], "read without EC05 command");
  UNNI_EXPECT_REJECTED(c.frames[0].status = FrameStatus::IncompleteByte; c.frame_errors = 1,
                       "structurally malformed command");
  UNNI_EXPECT_REJECTED(c.frames[1].status = FrameStatus::Truncated; c.frame_errors = 1,
                       "structurally malformed response");
#undef UNNI_EXPECT_REJECTED

  // Unrelated traffic may surround a valid exchange but must never itself
  // arm a measurement response.
  {
    Capture c{};
    c.frame_count = 3;
    c.frames[0].address = 0x40;
    c.frames[0].direction = Direction::Write;
    c.frames[0].address_ack = true;
    c.frames[0].length = 1;
    c.frames[0].data[0] = 0xAA;
    c.frames[0].ack[0] = true;
    c.frames[0].end_condition = EndCondition::Stop;
    c.frames[1] = pristine.frames[0];
    c.frames[2] = pristine.frames[1];
    co2_decoder::Result result{};
    if (!co2_decoder::process_capture(c, result) || result.co2_ppm != base.ppm) {
      ESP_LOGE(TAG, "CO2 hardening: unrelated traffic broke valid exchange");
      return false;
    }
  }

  // Deterministic structural mutation campaign. Every mutation is deliberately
  // validity-destroying; the assertion is therefore stronger than "no crash":
  // none may publish a CO2 value, and a clean frame must decode immediately
  // afterwards. ASan/UBSan runs this same campaign.
  uint32_t rng = 0x0601C02u;
  constexpr uint32_t MUTATIONS = 20000;
  for (uint32_t i = 0; i < MUTATIONS; i++) {
    rng = rng * 1664525u + 1013904223u;
    const Sample &sample = samples[(rng >> 8) % samples.size()];
    Capture c = make_capture(sample);
    switch (rng % 18u) {
      case 0: c.frames[1].data[0] ^= static_cast<uint8_t>(1u << ((rng >> 16) & 7u)); break;
      case 1: c.frames[1].data[1] ^= static_cast<uint8_t>(1u << ((rng >> 19) & 7u)); break;
      case 2: c.frames[1].data[2] ^= static_cast<uint8_t>(1u << ((rng >> 22) & 7u)); break;
      case 3: c.frames[1].length = 2; break;
      case 4: c.frames[1].length = 4; c.frames[1].data[3] = static_cast<uint8_t>(rng >> 24); break;
      case 5: c.frames[1].address ^= 0x01; break;
      case 6: c.frames[1].direction = Direction::Write; break;
      case 7: c.frames[1].address_ack = false; break;
      case 8: c.frames[1].ack[(rng >> 12) & 1u] = false; break;
      case 9: c.frames[1].ack[2] = true; break;
      case 10: c.frames[1].end_condition = EndCondition::CaptureEnd; break;
      case 11: c.frames[0].data[(rng >> 12) & 1u] ^= 0x01; break;
      case 12: c.frames[0].length = 1; break;
      case 13: c.frames[0].length = 3; c.frames[0].data[2] = 0; break;
      case 14: c.frames[0].address ^= 0x01; break;
      case 15: c.frames[0].address_ack = false; break;
      case 16: c.frames[0].ack[(rng >> 12) & 1u] = false; break;
      case 17: c.frames[0].end_condition = EndCondition::RepeatedStart; break;
    }
    co2_decoder::Result result{};
    co2_decoder::process_capture(c, result);
    if (result.have_co2) {
      ESP_LOGE(TAG, "CO2 hardening: deterministic mutation %u published %u ppm",
               static_cast<unsigned>(i), static_cast<unsigned>(result.co2_ppm));
      return false;
    }
  }

  ESP_LOGI(TAG, "CO2 hardening: %u current responses + %u deterministic mutations passed",
           static_cast<unsigned>(samples.size()), static_cast<unsigned>(MUTATIONS));
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

    const std::string &source = f[0];
    int ref_count = 0;
    float ref_period = NAN;
    float ref_duration = NAN;
    int rt_count = 0;
    float rt_period = NAN;
    float rt_duration = NAN;
    int rh_count = 0;
    float rh_period = NAN;
    float rh_duration = NAN;
    float expected_rt_ratio = NAN;
    float expected_rh_ratio = NAN;
    float expected_temperature = NAN;
    float expected_display_temperature = NAN;
    float expected_air_temperature = NAN;
    float expected_humidity = NAN;
    float expected_display_humidity = NAN;

    if (!parse_int(f[1], ref_count) || !parse_float(f[2], ref_period) || !parse_float(f[3], ref_duration) ||
        !parse_int(f[4], rt_count) || !parse_float(f[5], rt_period) || !parse_float(f[6], rt_duration) ||
        !parse_int(f[7], rh_count) || !parse_float(f[8], rh_period) || !parse_float(f[9], rh_duration) ||
        !parse_float(f[10], expected_rt_ratio) || !parse_float(f[11], expected_rh_ratio) ||
        !parse_float(f[12], expected_temperature) || !parse_float(f[13], expected_display_temperature) ||
        !parse_float(f[14], expected_air_temperature) || !parse_float(f[15], expected_humidity) ||
        !parse_float(f[16], expected_display_humidity)) {
      ESP_LOGE(TAG, "capture regression: parse error on line %u", static_cast<unsigned>(line_no));
      return false;
    }

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
  }

  if (tested < 100) {
    ESP_LOGE(TAG, "capture regression: only %u archived captures tested", static_cast<unsigned>(tested));
    return false;
  }

  ESP_LOGI(TAG, "capture regression: %u archived RT/RH captures passed (%u within air-temperature envelope)",
           static_cast<unsigned>(tested), static_cast<unsigned>(air_supported));
  return true;
}


bool CO2Monitor0601::run_current_capture_regression_tests_() {
  // Compact deterministic sample of the 2026-08-17/18 current-hardware
  // capture batch (two-wire RT/RH, 10 kOhm series resistors, carrier RH).
  // The raw archive contains 2227 timing records; this fixture keeps 256 valid
  // points spanning the full batch plus observed extrema/quantiles.
  const char *fixture_path = "tests/host/fixtures/rtrh_current_260817.csv";
  std::ifstream input(fixture_path);
  if (!input.is_open()) {
    ESP_LOGE(TAG, "current capture regression: cannot open %s", fixture_path);
    return false;
  }

  std::string line;
  if (!std::getline(input, line)) {
    ESP_LOGE(TAG, "current capture regression: empty fixture file");
    return false;
  }

  size_t tested = 0;
  size_t air_supported = 0;
  size_t extrapolated = 0;
  size_t line_no = 1;
  while (std::getline(input, line)) {
    line_no++;
    if (line.empty()) continue;
    const auto f = split_csv_line(line);
    if (f.size() != 17) {
      ESP_LOGE(TAG, "current capture regression: malformed line %u (%u fields)",
               static_cast<unsigned>(line_no), static_cast<unsigned>(f.size()));
      return false;
    }

    const std::string &received_at = f[0];
    int sequence = 0;
    float quality = NAN;
    float ref_period = NAN;
    float rt_period = NAN;
    float rh_period = NAN;
    int rh_count = 0;
    float expected_rt_ratio = NAN;
    float expected_rh_ratio = NAN;
    float captured_temperature = NAN;
    float captured_humidity = NAN;
    float expected_temperature = NAN;
    float expected_display_temperature = NAN;
    float expected_air_temperature = NAN;
    float expected_humidity = NAN;
    float expected_display_humidity = NAN;
    int expected_extrapolation = 0;

    if (!parse_int(f[1], sequence) || !parse_float(f[2], quality) ||
        !parse_float(f[3], ref_period) || !parse_float(f[4], rt_period) ||
        !parse_float(f[5], rh_period) || !parse_int(f[6], rh_count) ||
        !parse_float(f[7], expected_rt_ratio) || !parse_float(f[8], expected_rh_ratio) ||
        !parse_float(f[9], captured_temperature) || !parse_float(f[10], captured_humidity) ||
        !parse_float(f[11], expected_temperature) || !parse_float(f[12], expected_display_temperature) ||
        !parse_float(f[13], expected_air_temperature) || !parse_float(f[14], expected_humidity) ||
        !parse_float(f[15], expected_display_humidity) || !parse_int(f[16], expected_extrapolation)) {
      ESP_LOGE(TAG, "current capture regression: parse error on line %u", static_cast<unsigned>(line_no));
      return false;
    }

    if (received_at.empty() || sequence <= 0 || quality < 0.0f || quality > 100.0f ||
        ref_period <= 0.0f || rt_period <= 0.0f || rh_period <= 0.0f || rh_count < 120) {
      ESP_LOGE(TAG, "current capture regression: invalid measured input at %s seq=%d", received_at.c_str(), sequence);
      return false;
    }

    const float rt_ratio = rt_period / ref_period;
    const float rh_ratio = rh_period / ref_period;
    if (!nearly_equal(rt_ratio, expected_rt_ratio, 0.00002f) ||
        !nearly_equal(rh_ratio, expected_rh_ratio, 0.00002f)) {
      ESP_LOGE(TAG, "current capture regression: normalized ratio changed at %s seq=%d", received_at.c_str(), sequence);
      return false;
    }

    const float temperature = calibration::temperature_from_ratio(rt_ratio);
    const float display_temperature = calibration::display_temperature_from_ratio(rt_ratio);
    const float humidity = calibration::humidity_from_ratio_temperature(rh_ratio, temperature);
    const float display_humidity =
        calibration::display_humidity_from_ratio_temperature(rh_ratio, display_temperature);

    // The capture-time firmware values are rounded to 0.001 in rtrh_timing.csv.
    // Keep them as provenance checks in addition to the higher precision frozen
    // golden values generated from the same production calibration path.
    if (!nearly_equal(temperature, captured_temperature, 0.0011f) ||
        !nearly_equal(humidity, captured_humidity, 0.0011f)) {
      ESP_LOGE(TAG, "current capture regression: capture provenance mismatch at %s seq=%d", received_at.c_str(), sequence);
      return false;
    }
    if (!nearly_equal(temperature, expected_temperature, 0.002f) ||
        !nearly_equal(display_temperature, expected_display_temperature, 0.002f) ||
        !nearly_equal(humidity, expected_humidity, 0.003f) ||
        !nearly_equal(display_humidity, expected_display_humidity, 0.003f)) {
      ESP_LOGE(TAG, "current capture regression: calibration output changed at %s seq=%d", received_at.c_str(), sequence);
      return false;
    }

    const float air_temperature = calibration::air_temperature_from_ratio(rt_ratio);
    if (std::isfinite(expected_air_temperature)) {
      air_supported++;
      if (!nearly_equal(air_temperature, expected_air_temperature, 0.002f)) {
        ESP_LOGE(TAG, "current capture regression: air-temperature output changed at %s seq=%d", received_at.c_str(), sequence);
        return false;
      }
    } else if (std::isfinite(air_temperature)) {
      ESP_LOGE(TAG, "current capture regression: air-temperature envelope changed at %s seq=%d", received_at.c_str(), sequence);
      return false;
    }

    const bool extrapolation = calibration::is_extrapolation(temperature, rh_ratio);
    if (extrapolation != (expected_extrapolation != 0)) {
      ESP_LOGE(TAG, "current capture regression: extrapolation classification changed at %s seq=%d",
               received_at.c_str(), sequence);
      return false;
    }
    if (extrapolation) extrapolated++;
    tested++;
  }

  if (tested != 256) {
    ESP_LOGE(TAG, "current capture regression: expected 256 selected captures, got %u",
             static_cast<unsigned>(tested));
    return false;
  }

  ESP_LOGI(TAG,
           "current capture regression: %u current-hardware RT/RH captures passed (%u air-supported, %u extrapolated)",
           static_cast<unsigned>(tested), static_cast<unsigned>(air_supported), static_cast<unsigned>(extrapolated));
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
  // Use stderr for phase markers so host-test diagnostics remain visible even
  // when the ESPHome logger or stdout is buffered or not initialized yet.
  std::fprintf(stderr, "UNNI HOST SELF-TEST START\n");
  std::fflush(stderr);

  ESP_LOGI(TAG,
           "host smoke target: sniffer=%s rtrh=%s debug_metrics=%s light_sleep=%s BLE=%d BLE history=%d HA=%d",
           YESNO(this->sniffer_enabled_), YESNO(this->rtrh_enabled_), YESNO(this->debug_metrics_), YESNO(this->light_sleep_),
           UNNI_BLE_ENABLED, UNNI_BLE_HISTORY_ENABLED, UNNI_HOME_ASSISTANT_ENABLED);

  std::fprintf(stderr, "UNNI HOST SELF-TEST PHASE portable\n");
  std::fflush(stderr);
  if (!this->run_portable_self_test_()) {
    std::fprintf(stderr, "UNNI HOST SELF-TEST FAILED portable\n");
    std::fflush(stderr);
    this->mark_failed();
    return;
  }

  std::fprintf(stderr, "UNNI HOST SELF-TEST PHASE battery-policy\n");
  std::fflush(stderr);
  if (!this->run_battery_power_policy_tests_()) {
    std::fprintf(stderr, "UNNI HOST SELF-TEST FAILED battery-policy\n");
    std::fflush(stderr);
    this->mark_failed();
    return;
  }

  std::fprintf(stderr, "UNNI HOST SELF-TEST PHASE co2-hardening\n");
  std::fflush(stderr);
  if (!this->run_co2_decoder_hardening_tests_()) {
    std::fprintf(stderr, "UNNI HOST SELF-TEST FAILED co2-hardening\n");
    std::fflush(stderr);
    this->mark_failed();
    return;
  }

  std::fprintf(stderr, "UNNI HOST SELF-TEST PHASE captures\n");
  std::fflush(stderr);
  if (!this->run_capture_regression_tests_()) {
    std::fprintf(stderr, "UNNI HOST SELF-TEST FAILED captures\n");
    std::fflush(stderr);
    this->mark_failed();
    return;
  }

  std::fprintf(stderr, "UNNI HOST SELF-TEST PHASE current-captures\n");
  std::fflush(stderr);
  if (!this->run_current_capture_regression_tests_()) {
    std::fprintf(stderr, "UNNI HOST SELF-TEST FAILED current-captures\n");
    std::fflush(stderr);
    this->mark_failed();
    return;
  }

  std::fprintf(stderr, "UNNI HOST SELF-TEST PHASE publish\n");
  std::fflush(stderr);
  this->publish_fixture_values_();

  ESP_LOGI(TAG, "UNNI HOST SELF-TEST PASSED");
  std::fprintf(stderr, "UNNI HOST SELF-TEST PASSED\n");
  std::fflush(stderr);
}

}  // namespace co2_monitor_0601
}  // namespace esphome
