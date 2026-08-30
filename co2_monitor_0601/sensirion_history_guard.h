// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>

namespace esphome::co2_monitor_0601::sensirion_history_guard {

constexpr uint64_t CO2_DEFAULT_PERIOD_US = 6000000ULL;
constexpr uint64_t CO2_MIN_PERIOD_US = 4000000ULL;
constexpr uint64_t CO2_MAX_PERIOD_US = 8000000ULL;
constexpr uint64_t CO2_MAX_AGE_US = 30000000ULL;
constexpr uint64_t RTRH_DEFAULT_PERIOD_US = 30000000ULL;
constexpr uint64_t RTRH_MIN_PERIOD_US = 20000000ULL;
constexpr uint64_t RTRH_MAX_PERIOD_US = 40000000ULL;
constexpr uint64_t RTRH_MAX_AGE_US = 90000000ULL;
constexpr uint64_t RTRH_COMPLETION_LAG_US = 500000ULL;
constexpr uint64_t PRE_GUARD_US = 800000ULL;
constexpr uint64_t MIN_POST_GUARD_US = 150000ULL;
constexpr uint64_t MAX_POST_GUARD_US = 500000ULL;
constexpr uint64_t REACTIVE_MAX_US = 750000ULL;
constexpr uint64_t REACTIVE_TAIL_US = 25000ULL;
constexpr uint32_t DOWNLOAD_MAX_MS = 120000U;
constexpr uint32_t DOWNLOAD_NO_PROGRESS_MS = 15000U;

// Portable scheduling state. It is updated only from normal task context; the
// ISR-owned I2C and RT/RH state is sampled through read-only task probes.
struct Predictor {
  uint64_t last_frame_us{0};
  uint64_t estimated_period_us;
  uint64_t min_period_us;
  uint64_t max_period_us;
  uint64_t max_age_us;
  bool have_frame{false};

  constexpr Predictor(uint64_t period_us, uint64_t min_us, uint64_t max_us,
                      uint64_t age_us)
      : estimated_period_us(period_us), min_period_us(min_us),
        max_period_us(max_us), max_age_us(age_us) {}

  void note(uint64_t now_us) {
    if (have_frame && now_us > last_frame_us) {
      const uint64_t elapsed = now_us - last_frame_us;
      const uint64_t rounded_multiple =
          (elapsed + estimated_period_us / 2U) / estimated_period_us;
      const uint64_t multiple = std::clamp<uint64_t>(rounded_multiple, 1U, 8U);
      const uint64_t candidate = elapsed / multiple;
      if (candidate >= min_period_us && candidate <= max_period_us)
        estimated_period_us = (estimated_period_us * 7U + candidate) / 8U;
    }
    last_frame_us = now_us;
    have_frame = true;
  }

  bool blocked(uint64_t now_us) const {
    if (!have_frame || now_us < last_frame_us) return false;
    const uint64_t since_frame = now_us - last_frame_us;
    if (since_frame > max_age_us) return false;
    uint64_t periods = since_frame / estimated_period_us;
    if (periods == 0) periods = 1;
    uint64_t expected_us = last_frame_us + periods * estimated_period_us;
    if (now_us > expected_us + MAX_POST_GUARD_US)
      expected_us += estimated_period_us;
    if (now_us < expected_us)
      return expected_us - now_us <= PRE_GUARD_US;
    return now_us - expected_us <= MIN_POST_GUARD_US;
  }
};

struct Guard {
  Predictor co2{CO2_DEFAULT_PERIOD_US, CO2_MIN_PERIOD_US, CO2_MAX_PERIOD_US,
                CO2_MAX_AGE_US};
  Predictor rtrh{RTRH_DEFAULT_PERIOD_US, RTRH_MIN_PERIOD_US, RTRH_MAX_PERIOD_US,
                 RTRH_MAX_AGE_US};
  uint64_t reactive_since_us[2]{};
  uint64_t reactive_tail_until_us{0};
  uint8_t capture_mask{0};

  void note_co2_frame(uint64_t now_us) { co2.note(now_us); }
  void note_rtrh_cycle(uint64_t now_us) {
    rtrh.note(now_us >= RTRH_COMPLETION_LAG_US
                  ? now_us - RTRH_COMPLETION_LAG_US
                  : now_us);
  }

  void set_capture_mask(uint8_t mask, uint64_t now_us) {
    mask &= 0x03U;
    const uint8_t started = mask & ~capture_mask;
    const uint8_t ended = capture_mask & ~mask;
    for (uint8_t i = 0; i < 2; ++i) {
      const uint8_t bit = static_cast<uint8_t>(1U << i);
      if (started & bit) reactive_since_us[i] = now_us;
      if ((ended & bit) && now_us - reactive_since_us[i] <= REACTIVE_MAX_US)
        reactive_tail_until_us = now_us + REACTIVE_TAIL_US;
    }
    capture_mask = mask;
  }

  void set_capture_active(bool active, uint64_t now_us) {
    set_capture_mask(active ? 0x01U : 0x00U, now_us);
  }

  bool blocked(uint64_t now_us) const {
    for (uint8_t i = 0; i < 2; ++i)
      if ((capture_mask & (1U << i)) &&
          now_us - reactive_since_us[i] <= REACTIVE_MAX_US)
        return true;
    if (now_us < reactive_tail_until_us) return true;
    return co2.blocked(now_us) || rtrh.blocked(now_us);
  }
};

inline bool download_total_timeout(uint32_t now_ms, uint32_t started_ms) {
  return static_cast<uint32_t>(now_ms - started_ms) >= DOWNLOAD_MAX_MS;
}

inline bool download_no_progress_timeout(uint32_t now_ms, uint32_t last_progress_ms) {
  return static_cast<uint32_t>(now_ms - last_progress_ms) >= DOWNLOAD_NO_PROGRESS_MS;
}

}  // namespace esphome::co2_monitor_0601::sensirion_history_guard
