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

// RFC-1982-style serial comparison for generations whose distance is less
// than half the uint32_t range. This remains correct across UINT32_MAX -> 0.
inline bool generation_newer(uint32_t candidate, uint32_t reference) {
  return static_cast<int32_t>(candidate - reference) > 0;
}

}  // namespace esphome::co2_monitor_0601::sensirion_history_format
