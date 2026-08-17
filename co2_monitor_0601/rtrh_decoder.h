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
  bool temperature_valid{false};
  bool humidity_valid{false};

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
  float rh_carrier_period_us{NAN};
  uint16_t rh_carrier_count{0};
  float rh_carrier_ref_ratio{NAN};
  uint16_t rh_rt_rise_edges{0};
  uint16_t rh_rt_fall_edges{0};
  uint16_t rh_rh_rise_edges{0};
  uint16_t rh_rh_fall_edges{0};
  float rh_state_us{NAN};
  uint8_t rh_state_samples{0};
  uint32_t rh_state_seen{0};

  // Distribution diagnostics for the retained RH-state intervals. These do
  // not influence validation or humidity decoding; they only make timing
  // aliasing/missed-edge behaviour visible in the serial log.
  uint16_t rh_state_min_us{0};
  uint16_t rh_state_p25_us{0};
  uint16_t rh_state_p75_us{0};
  uint16_t rh_state_max_us{0};
  uint8_t rh_state_near_220{0};
  uint8_t rh_state_near_440{0};
  uint8_t rh_state_other{0};

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
bool setup(uint8_t rt_pin, uint8_t rh_pin);
void rearm_after_light_sleep();
void loop();
bool poll(Measurement &measurement);
void update_latest(const Measurement &measurement);

#if RTRH_DEBUG_CAPTURE
void register_debug_handlers();

// True while raw/timing debug data is still queued for UDP export.
bool debug_export_pending();
#endif

}  // namespace rtrh_decoder
}  // namespace co2_monitor_0601
}  // namespace esphome
