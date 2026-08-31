// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace esphome {
namespace co2_monitor_0601 {
namespace rtrh_decoder {

// A wake can occur after the native RT/RH waveform has already started. The
// first such cycle is deliberately discarded. Once its terminating idle gap
// has been observed, the next first edge is known to belong to a fresh cycle
// and can safely establish the time-based REF/RT/RH phase origin.
enum class WakeCaptureState : uint8_t {
  UNSYNCHRONIZED = 0,
  DISCARD_UNTIL_IDLE,
  FRESH_CYCLE_ARMED,
  FRESH_CYCLE_ACTIVE,
};

enum class WakeRearmAction : uint8_t {
  NONE = 0,
  DISCARD_PARTIAL,
  KEEP_FRESH_CYCLE,
};

constexpr bool wake_edge_is_discarded(WakeCaptureState state) {
  return state == WakeCaptureState::DISCARD_UNTIL_IDLE;
}

constexpr WakeCaptureState wake_state_after_edge(WakeCaptureState state) {
  return state == WakeCaptureState::FRESH_CYCLE_ARMED
             ? WakeCaptureState::FRESH_CYCLE_ACTIVE
             : state;
}

constexpr WakeRearmAction wake_rearm_action(WakeCaptureState state, bool collecting) {
  if (!collecting) return WakeRearmAction::NONE;
  return state == WakeCaptureState::FRESH_CYCLE_ACTIVE
             ? WakeRearmAction::KEEP_FRESH_CYCLE
             : WakeRearmAction::DISCARD_PARTIAL;
}

constexpr WakeCaptureState wake_state_after_rearm(WakeCaptureState state, bool collecting) {
  const WakeRearmAction action = wake_rearm_action(state, collecting);
  if (action == WakeRearmAction::KEEP_FRESH_CYCLE) return WakeCaptureState::UNSYNCHRONIZED;
  if (action == WakeRearmAction::DISCARD_PARTIAL) return WakeCaptureState::DISCARD_UNTIL_IDLE;
  return state;
}

constexpr WakeCaptureState wake_state_after_idle(WakeCaptureState state) {
  return state == WakeCaptureState::DISCARD_UNTIL_IDLE
             ? WakeCaptureState::FRESH_CYCLE_ARMED
             : state;
}

constexpr WakeCaptureState wake_state_after_complete_cycle() {
  return WakeCaptureState::FRESH_CYCLE_ARMED;
}

}  // namespace rtrh_decoder
}  // namespace co2_monitor_0601
}  // namespace esphome
