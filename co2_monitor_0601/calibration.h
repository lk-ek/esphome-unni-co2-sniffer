// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cmath>
#include <limits>

namespace esphome {
namespace co2_monitor_0601 {
namespace calibration {

/*
 * RT/RH calibration model
 * -----------------------
 *
 * The edge decoder deliberately knows nothing about calibration coefficients.
 * It measures two normalized quantities:
 *
 *   rt_ratio = RT_period / REF_period
 *   rh_ratio = RH_state_period / REF_period
 *
 * This module converts those ratios to engineering units.  Keeping the model
 * here means a future refit only changes this file, not the timing decoder.
 */

// Temperature calibration.
//   T [degC] = M * rt_ratio + C
//
// The slope is retained from the 2026-08-11 fit.  The intercept is
// provisionally re-anchored for the final two-wire RT/RH hookup with 10 kOhm
// series resistors, using the 2026-08-16 stationary captures #6..#8:
//   mean(rt_ratio) = 1.992839 -> Unni display = 24.0 degC
//
// This is deliberately a one-point correction until a new temperature sweep
// is available; changing the slope from a single point would be unjustified.
inline constexpr float TEMP_RATIO_M = -23.024269f;
inline constexpr float TEMP_RATIO_C = 69.883663f;

// Relative-humidity calibration (v4):
//   x  = ln(rh_ratio)
//   RH = A*x^2 + B*x + C*T + D
//
// These are intentionally unchanged from the validated v4 model.
inline constexpr float RH_LOG2_A = 6.11947870f;
inline constexpr float RH_LOG_B = -33.93748066f;
inline constexpr float RH_TEMP_C = -0.48564674f;
inline constexpr float RH_OFFSET = 93.38516444f;

// Approximate stationary calibration envelope currently backed by measurements.
// Values outside this envelope are still converted; the caller may expose the
// extrapolation flag diagnostically.
inline constexpr float CAL_TEMP_MIN_C = 18.0f;
inline constexpr float CAL_TEMP_MAX_C = 24.0f;
inline constexpr float CAL_RH_RATIO_MIN = 3.20f;
inline constexpr float CAL_RH_RATIO_MAX = 8.20f;

inline float temperature_from_ratio(float rt_ratio) {
  return TEMP_RATIO_M * rt_ratio + TEMP_RATIO_C;
}

inline float log_rh_ratio(float rh_ratio) {
  if (!(rh_ratio > 0.0f))
    return std::numeric_limits<float>::quiet_NaN();
  return std::log(rh_ratio);
}

inline float humidity_from_ratio_temperature(float rh_ratio,
                                              float temperature_c) {
  const float x = log_rh_ratio(rh_ratio);
  if (!std::isfinite(x) || !std::isfinite(temperature_c))
    return std::numeric_limits<float>::quiet_NaN();

  float rh = RH_LOG2_A * x * x + RH_LOG_B * x +
             RH_TEMP_C * temperature_c + RH_OFFSET;

  if (rh < 0.0f)
    rh = 0.0f;
  else if (rh > 100.0f)
    rh = 100.0f;

  return rh;
}

inline bool is_extrapolation(float temperature_c, float rh_ratio) {
  if (!std::isfinite(temperature_c) || !std::isfinite(rh_ratio))
    return true;

  return temperature_c < CAL_TEMP_MIN_C ||
         temperature_c > CAL_TEMP_MAX_C ||
         rh_ratio < CAL_RH_RATIO_MIN ||
         rh_ratio > CAL_RH_RATIO_MAX;
}

}  // namespace calibration
}  // namespace co2_monitor_0601
}  // namespace esphome
