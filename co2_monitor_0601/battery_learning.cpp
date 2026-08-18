// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "battery_learning.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace co2_monitor_0601 {
namespace battery_learning {

namespace {
constexpr float MIN_SESSION_H = 2.0f;
constexpr float MIN_DROP_PERCENT = 8.0f;
constexpr float MIN_FULL_RUNTIME_H = 5.0f;
constexpr float MAX_FULL_RUNTIME_H = 200.0f;
constexpr float LEARN_ALPHA = 0.25f;
}  // namespace

float progress_percent(const State &state) {
  if (!state.session_active || !std::isfinite(state.session_start_progress) ||
      !std::isfinite(state.session_last_progress))
    return 0.0f;

  const float drop = std::max(0.0f, state.session_start_progress - state.session_last_progress);
  const float time_factor = std::min(1.0f, state.session_elapsed_ms / (MIN_SESSION_H * 3600000.0f));
  const float drop_factor = std::min(1.0f, drop / MIN_DROP_PERCENT);
  return 100.0f * std::min(time_factor, drop_factor);
}

void advance_elapsed(State &state, uint32_t now) {
  if (!state.session_active || state.last_tick_ms == 0) return;
  state.session_elapsed_ms += static_cast<uint32_t>(now - state.last_tick_ms);
  state.last_tick_ms = now;
}

void update(State &state, float progress, uint32_t now) {
  if (!state.session_active) {
    state.session_active = true;
    state.session_start_progress = progress;
    state.session_last_progress = progress;
    state.session_elapsed_ms = 0;
    state.last_tick_ms = now;
    return;
  }

  advance_elapsed(state, now);
  state.session_last_progress = progress;
}

FinalizeResult finalize(State &state, bool completed_session) {
  FinalizeResult result{};
  if (!state.session_active) return result;

  const float drop = state.session_start_progress - state.session_last_progress;
  const float elapsed_h = state.session_elapsed_ms / 3600000.0f;

  if (completed_session && elapsed_h >= MIN_SESSION_H && drop >= MIN_DROP_PERCENT) {
    const float observed_full_h = elapsed_h * 100.0f / drop;
    result.observed_full_runtime_h = observed_full_h;
    if (std::isfinite(observed_full_h) && observed_full_h >= MIN_FULL_RUNTIME_H &&
        observed_full_h <= MAX_FULL_RUNTIME_H) {
      state.learned_full_runtime_h =
          std::isfinite(state.learned_full_runtime_h)
              ? (LEARN_ALPHA * observed_full_h + (1.0f - LEARN_ALPHA) * state.learned_full_runtime_h)
              : observed_full_h;
      if (state.learned_cycles < 65535U) state.learned_cycles++;
      result.model_updated = true;
    }
  }

  state.session_active = false;
  state.session_elapsed_ms = 0;
  state.session_start_progress = NAN;
  state.session_last_progress = NAN;
  state.last_tick_ms = 0;
  return result;
}

}  // namespace battery_learning
}  // namespace co2_monitor_0601
}  // namespace esphome
