// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace esphome::co2_monitor_0601 {

// Optional producer-side protection for history notifications. The history
// transport owns pausing, cursor retention and watchdogs; a measurement
// producer only reports whether a notification would currently be disruptive.
class HistoryTransferGuard {
 public:
  virtual ~HistoryTransferGuard() = default;
  virtual bool blocked(uint64_t now_us) = 0;
  virtual void set_download_active(bool active) { (void) active; }
};

}  // namespace esphome::co2_monitor_0601
