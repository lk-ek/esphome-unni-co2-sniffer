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

// Rebase a bounded continuous-run window so that newest_epoch_s describes its
// newest retained sample. This is used when the 4096-sample ring evicts its
// oldest entry; keeping the original anchor would make restored history older
// by one interval for every later overwrite.
inline uint32_t run_anchor_for_latest_epoch_s(uint32_t newest_epoch_s, uint16_t run_count,
                                               uint32_t interval_ms) {
  if (!wall_clock_valid(newest_epoch_s) || run_count == 0) return 0;
  const uint32_t span_s = static_cast<uint32_t>(
      (static_cast<uint64_t>(run_count - 1U) * interval_ms) / 1000ULL);
  return newest_epoch_s >= span_s ? newest_epoch_s - span_s : 0;
}

// Fallback for a temporarily unavailable wall clock. Metadata only stores
// second-resolution anchors, so preserve the best monotonic approximation until
// a later valid wall-clock sample can rebase the window exactly.
inline uint32_t advance_run_anchor_epoch_s(uint32_t anchor_epoch_s, uint32_t interval_ms) {
  if (!wall_clock_valid(anchor_epoch_s)) return 0;
  return anchor_epoch_s + interval_ms / 1000U;
}

// RFC-1982-style serial comparison for generations whose distance is less
// than half the uint32_t range. This remains correct across UINT32_MAX -> 0.
inline bool generation_newer(uint32_t candidate, uint32_t reference) {
  return static_cast<int32_t>(candidate - reference) > 0;
}

inline bool within_elapsed_window(uint32_t now, uint32_t anchor, uint32_t window_ms) {
  return static_cast<uint32_t>(now - anchor) < window_ms;
}

}  // namespace esphome::co2_monitor_0601::sensirion_history_format
