// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <cmath>

namespace esphome {
namespace co2_monitor_0601 {
namespace battery_learning {

struct State {
  bool session_active{false};
  uint32_t session_elapsed_ms{0};
  uint32_t last_tick_ms{0};
  float session_start_progress{NAN};
  float session_last_progress{NAN};
  float learned_full_runtime_h{NAN};
  uint16_t learned_cycles{0};
};

struct FinalizeResult {
  bool model_updated{false};
  float observed_full_runtime_h{NAN};
};

float progress_percent(const State &state);
void update(State &state, float progress, uint32_t now);
void advance_elapsed(State &state, uint32_t now);
FinalizeResult finalize(State &state, bool completed_session);

}  // namespace battery_learning
}  // namespace co2_monitor_0601
}  // namespace esphome
