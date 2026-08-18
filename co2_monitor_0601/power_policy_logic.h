// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace esphome {
namespace co2_monitor_0601 {
namespace power_policy_logic {

struct State {
  bool energy_save_mode{false};
  bool policy_active{false};
  bool grace_pending{false};
  uint32_t grace_started_ms{0};
  uint32_t grace_ms{3000};
};

enum class SetModeAction : uint8_t {
  NoChange,
  ApplyUsbPolicy,
  StartGrace,
  ApplyBatteryPolicy,
};

inline bool external_powered(bool usb_powered, const State &state) {
  return usb_powered && !state.policy_active;
}

inline SetModeAction set_mode(State &state, bool enabled, bool usb_powered, uint32_t now) {
  if (state.energy_save_mode == enabled) return SetModeAction::NoChange;

  state.energy_save_mode = enabled;
  if (!enabled) {
    state.grace_pending = false;
    state.policy_active = false;
    return SetModeAction::ApplyUsbPolicy;
  }

  if (usb_powered && state.grace_ms > 0) {
    state.policy_active = false;
    state.grace_pending = true;
    state.grace_started_ms = now;
    return SetModeAction::StartGrace;
  }

  state.grace_pending = false;
  state.policy_active = true;
  return SetModeAction::ApplyBatteryPolicy;
}

inline bool process_grace(State &state, uint32_t now) {
  if (!state.grace_pending) return false;
  if (!state.energy_save_mode) {
    state.grace_pending = false;
    return false;
  }
  if (static_cast<uint32_t>(now - state.grace_started_ms) < state.grace_ms) return false;

  state.grace_pending = false;
  state.policy_active = true;
  return true;
}

inline uint32_t ble_advertising_interval(bool external_power, uint32_t usb_interval_ms,
                                         uint32_t battery_interval_ms) {
  return external_power ? usb_interval_ms : battery_interval_ms;
}

}  // namespace power_policy_logic
}  // namespace co2_monitor_0601
}  // namespace esphome
