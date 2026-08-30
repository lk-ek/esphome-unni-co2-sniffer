// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "../sensirion_gadget_bridge/history_transfer_guard.h"

#include <algorithm>
#include <cstdint>

namespace esphome::co2_monitor_0601 {

class UnniHistoryTransferGuard : public HistoryTransferGuard {
 public:
  using CaptureProbe = uint8_t (*)();

  void set_capture_probe(CaptureProbe probe) { this->capture_probe_ = probe; }
  void note_co2_frame(uint64_t now_us) { this->note_valid_co2_frame(now_us); }
  void note_valid_co2_frame(uint64_t now_us) { this->co2_.note(now_us); }
  void note_rtrh_cycle(uint64_t now_us) {
    this->rtrh_.note(now_us >= RTRH_COMPLETION_LAG_US ? now_us - RTRH_COMPLETION_LAG_US : now_us);
  }
  bool note_co2_capture(uint16_t raw_scl_edges, bool frame_error) {
    if (!this->download_active_) return false;
    const uint64_t previous = this->pre_guard_us();
    this->note_capture_quality_(raw_scl_edges < 130U || frame_error);
    return previous != this->pre_guard_us();
  }
  void set_capture_mask(uint8_t mask, uint64_t now_us) { this->set_capture_mask_(mask, now_us); }
  void set_capture_active(bool active, uint64_t now_us) {
    this->set_capture_mask_(active ? 0x01U : 0x00U, now_us);
  }
  void note_capture_quality(bool damaged) { this->note_capture_quality_(damaged); }
  uint64_t pre_guard_us() const { return PRE_GUARD_US + this->pre_guard_extra_us_; }
  uint64_t co2_estimated_period_us() const { return this->co2_.estimated_period_us; }

  static constexpr uint64_t rtrh_completion_lag_us() { return RTRH_COMPLETION_LAG_US; }

  bool blocked(uint64_t now_us) override {
    // Without a producer probe, retain explicitly supplied state. This keeps
    // the portable scheduler independently testable and avoids inventing a
    // capture-end transition (and reactive tail) merely because no probe was
    // installed. Production Unni composition always installs the probe.
    if (this->capture_probe_ != nullptr)
      this->set_capture_mask_(this->capture_probe_(), now_us);
    for (uint8_t i = 0; i < 2; ++i)
      if ((this->capture_mask_ & (1U << i)) &&
          now_us - this->reactive_since_us_[i] <= REACTIVE_MAX_US)
        return true;
    if (now_us < this->reactive_tail_until_us_) return true;
    return this->co2_.blocked(now_us, this->pre_guard_us()) ||
           this->rtrh_.blocked(now_us, this->pre_guard_us());
  }

  void set_download_active(bool active) override {
    this->download_active_ = active;
    if (!active) this->clean_capture_streak_ = 0;
  }

 protected:
  static constexpr uint64_t CO2_DEFAULT_PERIOD_US = 6000000ULL;
  static constexpr uint64_t CO2_MIN_PERIOD_US = 4000000ULL;
  static constexpr uint64_t CO2_MAX_PERIOD_US = 8000000ULL;
  static constexpr uint64_t CO2_MAX_AGE_US = 30000000ULL;
  static constexpr uint64_t RTRH_DEFAULT_PERIOD_US = 30000000ULL;
  static constexpr uint64_t RTRH_MIN_PERIOD_US = 20000000ULL;
  static constexpr uint64_t RTRH_MAX_PERIOD_US = 40000000ULL;
  static constexpr uint64_t RTRH_MAX_AGE_US = 90000000ULL;
  static constexpr uint64_t RTRH_COMPLETION_LAG_US = 500000ULL;
  static constexpr uint64_t PRE_GUARD_US = 800000ULL;
  static constexpr uint64_t PRE_GUARD_DAMAGE_STEP_US = 250000ULL;
  static constexpr uint64_t PRE_GUARD_MAX_EXTRA_US = 500000ULL;
  static constexpr uint64_t PRE_GUARD_RECOVERY_STEP_US = 50000ULL;
  static constexpr uint8_t CLEAN_CAPTURES_PER_RECOVERY = 3;
  static constexpr uint64_t MIN_POST_GUARD_US = 150000ULL;
  static constexpr uint64_t MAX_POST_GUARD_US = 500000ULL;
  static constexpr uint64_t REACTIVE_MAX_US = 750000ULL;
  static constexpr uint64_t REACTIVE_TAIL_US = 25000ULL;

  struct Predictor {
    uint64_t last_frame_us{0};
    uint64_t estimated_period_us;
    uint64_t min_period_us;
    uint64_t max_period_us;
    uint64_t max_age_us;
    bool have_frame{false};

    constexpr Predictor(uint64_t period, uint64_t minimum, uint64_t maximum, uint64_t age)
        : estimated_period_us(period), min_period_us(minimum), max_period_us(maximum), max_age_us(age) {}

    void note(uint64_t now_us) {
      if (have_frame && now_us > last_frame_us) {
        const uint64_t elapsed = now_us - last_frame_us;
        const uint64_t rounded = (elapsed + estimated_period_us / 2U) / estimated_period_us;
        const uint64_t multiple = std::clamp<uint64_t>(rounded, 1U, 8U);
        const uint64_t candidate = elapsed / multiple;
        if (candidate >= min_period_us && candidate <= max_period_us)
          estimated_period_us = (estimated_period_us * 7U + candidate) / 8U;
      }
      last_frame_us = now_us;
      have_frame = true;
    }

    bool blocked(uint64_t now_us, uint64_t pre_guard_us) const {
      if (!have_frame || now_us < last_frame_us) return false;
      const uint64_t since = now_us - last_frame_us;
      if (since > max_age_us) return false;
      uint64_t periods = since / estimated_period_us;
      if (periods == 0) periods = 1;
      uint64_t expected = last_frame_us + periods * estimated_period_us;
      if (now_us > expected + MAX_POST_GUARD_US) expected += estimated_period_us;
      if (now_us < expected) return expected - now_us <= pre_guard_us;
      return now_us - expected <= MIN_POST_GUARD_US;
    }
  };

  void set_capture_mask_(uint8_t mask, uint64_t now_us) {
    mask &= 0x03U;
    const uint8_t started = mask & ~this->capture_mask_;
    const uint8_t ended = this->capture_mask_ & ~mask;
    for (uint8_t i = 0; i < 2; ++i) {
      const uint8_t bit = static_cast<uint8_t>(1U << i);
      if (started & bit) this->reactive_since_us_[i] = now_us;
      if ((ended & bit) && now_us - this->reactive_since_us_[i] <= REACTIVE_MAX_US)
        this->reactive_tail_until_us_ = now_us + REACTIVE_TAIL_US;
    }
    this->capture_mask_ = mask;
  }

  void note_capture_quality_(bool damaged) {
    if (damaged) {
      this->pre_guard_extra_us_ = std::min<uint64_t>(
          this->pre_guard_extra_us_ + PRE_GUARD_DAMAGE_STEP_US, PRE_GUARD_MAX_EXTRA_US);
      this->clean_capture_streak_ = 0;
      return;
    }
    if (this->pre_guard_extra_us_ == 0) return;
    if (++this->clean_capture_streak_ < CLEAN_CAPTURES_PER_RECOVERY) return;
    this->clean_capture_streak_ = 0;
    this->pre_guard_extra_us_ = this->pre_guard_extra_us_ > PRE_GUARD_RECOVERY_STEP_US
                                    ? this->pre_guard_extra_us_ - PRE_GUARD_RECOVERY_STEP_US
                                    : 0;
  }

  Predictor co2_{CO2_DEFAULT_PERIOD_US, CO2_MIN_PERIOD_US, CO2_MAX_PERIOD_US, CO2_MAX_AGE_US};
  Predictor rtrh_{RTRH_DEFAULT_PERIOD_US, RTRH_MIN_PERIOD_US, RTRH_MAX_PERIOD_US, RTRH_MAX_AGE_US};
  CaptureProbe capture_probe_{nullptr};
  uint64_t reactive_since_us_[2]{};
  uint64_t reactive_tail_until_us_{0};
  uint64_t pre_guard_extra_us_{0};
  uint8_t capture_mask_{0};
  uint8_t clean_capture_streak_{0};
  bool download_active_{false};
};

}  // namespace esphome::co2_monitor_0601
