// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace esphome {
namespace co2_monitor_0601 {
namespace rtrh_decoder {

// A complete native RH burst is not fixed at the historical ~131 ms. Battery
// captures from the final two-wire/10 kohm installation span down to 115.8 ms
// while retaining balanced RT/RH edges and hundreds of carrier cycles. Keep a
// margin below that observed envelope while still rejecting truncated tails.
inline constexpr float RH_DURATION_MIN_MS = 110.0f;
inline constexpr float RH_DURATION_MAX_MS = 134.0f;

inline constexpr bool rh_duration_is_plausible(float duration_ms) {
  return duration_ms >= RH_DURATION_MIN_MS && duration_ms <= RH_DURATION_MAX_MS;
}

}  // namespace rtrh_decoder
}  // namespace co2_monitor_0601
}  // namespace esphome
