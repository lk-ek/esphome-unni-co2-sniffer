// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "esphome/core/defines.h"

#include <cmath>
#include <cstdint>

#ifndef RTRH_DEBUG_CAPTURE
#define RTRH_DEBUG_CAPTURE 0
#endif

namespace esphome {
namespace co2_monitor_0601 {
namespace rtrh_decoder {

enum class RejectReason : uint8_t {
  NONE = 0,
  REF_PERIOD,
  REF_DURATION,
  RT_PERIOD,
  RT_DURATION,
  RT_COUNT,
  RH_DURATION,
  RH_TOO_FEW_SAMPLES,
  RH_STATE_PERIOD,
  RH_RATIO_IMPLAUSIBLE,
};

const char *reject_reason_to_string(RejectReason reason);

struct Measurement {
  uint32_t sequence{0};
  bool valid{false};

  // Timing diagnostics retained for optional ESPHome entities and CSV debug.
  float ref_period_us{NAN};
  float ref_duration_ms{NAN};
  uint16_t ref_count{0};
  float rt_phase_period_us{NAN};
  float rt_duration_ms{NAN};
  uint16_t rt_phase_count{0};
  float rt_period_us{NAN};
  uint16_t rt_count{0};
  float rh_duration_ms{NAN};
  float rh_state_us{NAN};
  uint8_t rh_state_samples{0};
  uint32_t rh_state_seen{0};
  uint32_t rh_irq_rt{0};
  uint32_t rh_irq_rh{0};
  uint32_t rh_state_00{0};
  uint32_t rh_state_01{0};
  uint32_t rh_state_08{0};
  uint32_t rh_state_09{0};
  uint32_t rh_rise_pairs{0};
  uint32_t rh_fall_pairs{0};
  uint32_t rh_rise_rt_first{0};
  uint32_t rh_rise_rh_first{0};
  uint32_t rh_fall_rt_first{0};
  uint32_t rh_fall_rh_first{0};
  float rh_rise_skew_mean_us{NAN};
  float rh_fall_skew_mean_us{NAN};

  float rt_ratio{NAN};
  float rh_ratio{NAN};
  float rh_log{NAN};
  float temperature_c{NAN};
  float humidity_percent{NAN};
  float quality_percent{0.0f};
  RejectReason reject_reason{RejectReason::NONE};

  bool thermal_transient{false};
  bool temperature_extrapolation{false};
  bool humidity_extrapolation{false};
  bool calibration_extrapolation{false};
};

// GPIO3/D1 = RT, GPIO4/D2 = RH. D3/GPIO5 is not required.
bool setup(uint8_t rt_pin, uint8_t rh_pin, bool enable_edge_isr = true);
void loop();
bool poll(Measurement &measurement);
void update_latest(const Measurement &measurement);

#if RTRH_DEBUG_CAPTURE
void register_debug_handlers();
#endif

}  // namespace rtrh_decoder
}  // namespace co2_monitor_0601
}  // namespace esphome
