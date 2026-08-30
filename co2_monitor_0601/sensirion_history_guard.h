// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>

namespace esphome::co2_monitor_0601::sensirion_history_guard {

constexpr uint64_t DEFAULT_PERIOD_US = 6000000ULL;
constexpr uint64_t MIN_PERIOD_US = 4000000ULL;
constexpr uint64_t MAX_PERIOD_US = 8000000ULL;
constexpr uint64_t PRE_GUARD_US = 300000ULL;
constexpr uint64_t MIN_POST_GUARD_US = 150000ULL;
constexpr uint64_t MAX_POST_GUARD_US = 500000ULL;
constexpr uint64_t REACTIVE_MAX_US = 750000ULL;
constexpr uint64_t REACTIVE_TAIL_US = 25000ULL;
constexpr uint64_t PREDICTION_MAX_AGE_US = 30000000ULL;
constexpr uint32_t DOWNLOAD_MAX_MS = 120000U;
constexpr uint32_t DOWNLOAD_NO_PROGRESS_MS = 15000U;

// Portable scheduling state. It is updated only from normal task context; the
// ISR-owned I2C state is sampled through i2c_sniffer::capture_in_progress().
struct Guard {
  uint64_t last_frame_us{0};
  uint64_t estimated_period_us{DEFAULT_PERIOD_US};
  uint64_t reactive_since_us{0};
  uint64_t reactive_tail_until_us{0};
  bool have_frame{false};
  bool capture_active{false};

  void note_valid_frame(uint64_t now_us) {
    if (have_frame && now_us > last_frame_us) {
      const uint64_t elapsed = now_us - last_frame_us;
      const uint64_t rounded_multiple =
          (elapsed + estimated_period_us / 2U) / estimated_period_us;
      const uint64_t multiple = std::clamp<uint64_t>(rounded_multiple, 1U, 8U);
      const uint64_t candidate = elapsed / multiple;
      if (candidate >= MIN_PERIOD_US && candidate <= MAX_PERIOD_US)
        estimated_period_us = (estimated_period_us * 7U + candidate) / 8U;
    }
    last_frame_us = now_us;
    have_frame = true;
  }

  void set_capture_active(bool active, uint64_t now_us) {
    if (active == capture_active) return;
    capture_active = active;
    if (active) {
      reactive_since_us = now_us;
      reactive_tail_until_us = 0;
    } else if (now_us - reactive_since_us <= REACTIVE_MAX_US) {
      reactive_tail_until_us = now_us + REACTIVE_TAIL_US;
    }
  }

  bool predictive_blocked(uint64_t now_us) const {
    if (!have_frame || now_us < last_frame_us) return false;

    const uint64_t since_frame = now_us - last_frame_us;
    if (since_frame > PREDICTION_MAX_AGE_US) return false;
    uint64_t periods = since_frame / estimated_period_us;
    if (periods == 0) periods = 1;
    uint64_t expected_us = last_frame_us + periods * estimated_period_us;
    if (now_us > expected_us + MAX_POST_GUARD_US)
      expected_us += estimated_period_us;

    if (now_us < expected_us)
      return expected_us - now_us <= PRE_GUARD_US;
    return now_us - expected_us <= MIN_POST_GUARD_US;
  }

  bool blocked(uint64_t now_us) const {
    if (capture_active && now_us - reactive_since_us <= REACTIVE_MAX_US)
      return true;
    if (!capture_active && now_us < reactive_tail_until_us)
      return true;
    return predictive_blocked(now_us);
  }
};

inline bool download_total_timeout(uint32_t now_ms, uint32_t started_ms) {
  return static_cast<uint32_t>(now_ms - started_ms) >= DOWNLOAD_MAX_MS;
}

inline bool download_no_progress_timeout(uint32_t now_ms, uint32_t last_progress_ms) {
  return static_cast<uint32_t>(now_ms - last_progress_ms) >= DOWNLOAD_NO_PROGRESS_MS;
}

}  // namespace esphome::co2_monitor_0601::sensirion_history_guard
