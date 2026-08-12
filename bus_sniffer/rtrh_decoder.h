// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "measurement_quality.h"
#include <cstdint>
#include <cmath>

namespace esphome {
namespace bus_sniffer {
namespace rtrh_decoder {

struct Accum {
  uint32_t period_sum{0};
  uint16_t count{0};
};

static constexpr uint8_t RH_STATE_PERIOD_SAMPLES = 96;
struct RhStateStats {
  uint32_t last_us{0};
  uint16_t samples[RH_STATE_PERIOD_SAMPLES]{};
  uint8_t write_pos{0};
  uint8_t sample_count{0};
  uint32_t seen{0};
};

struct Snapshot {
  Accum ref;
  Accum rt;
  Accum rh_timing;
  uint32_t rt_temp_period_sum{0};
  uint16_t rt_temp_count{0};
  RhStateStats rh_state;
  uint32_t sequence{0};
};

struct Derived {
  uint32_t sequence{0};
  bool valid{false};
  float rt_ratio{NAN};
  float rh_ratio{NAN};
  float temperature_c{NAN};
  float humidity_percent{NAN};
  float quality_percent{0.0f};
  measurement_quality::RejectReason reject_reason{measurement_quality::RejectReason::NONE};
  bool thermal_transient{false};
  bool temperature_extrapolation{false};
  bool humidity_extrapolation{false};
  bool calibration_extrapolation{false};
};

// GPIO3/D1 = G10, GPIO5/D3 = G11, GPIO4/D2 = G13.
bool setup();
void loop();
bool poll(Snapshot &snapshot);
float rh_state_period_median(const RhStateStats &stats);
void set_derived(const Derived &derived);

#if RTRH_DEBUG_CAPTURE
void register_debug_handlers();
#endif

}  // namespace rtrh_decoder
}  // namespace bus_sniffer
}  // namespace esphome
