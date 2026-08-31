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

  // Optional producer diagnostics included in transfer-completion logs. They
  // deliberately do not affect the generic scheduler or wire protocol.
  virtual uint32_t diagnostic_capture_count() const { return 0; }
  virtual uint32_t diagnostic_damaged_capture_count() const { return 0; }
  virtual uint16_t diagnostic_min_raw_edges() const { return 0; }
  virtual uint32_t diagnostic_pre_guard_ms() const { return 0; }
};

}  // namespace esphome::co2_monitor_0601
