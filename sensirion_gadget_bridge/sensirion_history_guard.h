// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>

namespace esphome::co2_monitor_0601::sensirion_history_guard {

constexpr uint32_t DOWNLOAD_MAX_MS = 120000U;
constexpr uint32_t DOWNLOAD_NO_PROGRESS_MS = 15000U;

inline bool download_total_timeout(uint32_t now_ms, uint32_t started_ms) {
  return static_cast<uint32_t>(now_ms - started_ms) >= DOWNLOAD_MAX_MS;
}

inline bool download_no_progress_timeout(uint32_t now_ms, uint32_t last_progress_ms) {
  return static_cast<uint32_t>(now_ms - last_progress_ms) >= DOWNLOAD_NO_PROGRESS_MS;
}

}  // namespace esphome::co2_monitor_0601::sensirion_history_guard
