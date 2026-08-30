// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace esphome::co2_monitor_0601::sensirion_history_format {

constexpr uint32_t MIN_INTERVAL_MS = 60000;
constexpr uint32_t MAX_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL;

inline bool interval_valid(uint32_t interval_ms) {
  return interval_ms >= MIN_INTERVAL_MS && interval_ms <= MAX_INTERVAL_MS;
}

constexpr uint32_t MIN_VALID_EPOCH_S = 1577836800UL;  // 2020-01-01 UTC

inline bool wall_clock_valid(uint32_t epoch_s) {
  return epoch_s >= MIN_VALID_EPOCH_S;
}

// A sample cadence gap exists only after at least one complete interval was
// missed. Normal loop jitter does not create a new history run.
inline bool cadence_gap(uint32_t elapsed_ms, uint32_t interval_ms) {
  return interval_ms > 0 && elapsed_ms >= interval_ms * 2ULL;
}

inline uint32_t run_latest_epoch_s(uint32_t anchor_epoch_s, uint16_t run_count,
                                   uint32_t interval_ms) {
  if (!wall_clock_valid(anchor_epoch_s) || run_count == 0) return 0;
  return anchor_epoch_s +
         static_cast<uint32_t>((static_cast<uint64_t>(run_count - 1U) * interval_ms) / 1000ULL);
}

// RFC-1982-style serial comparison for generations whose distance is less
// than half the uint32_t range. This remains correct across UINT32_MAX -> 0.
inline bool generation_newer(uint32_t candidate, uint32_t reference) {
  return static_cast<int32_t>(candidate - reference) > 0;
}

}  // namespace esphome::co2_monitor_0601::sensirion_history_format
