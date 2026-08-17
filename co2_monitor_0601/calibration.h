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
 *   rh_ratio = RH_carrier_period / REF_period
 *
 * This module converts those ratios to engineering units.  Keeping the model
 * here means a future refit only changes this file, not the timing decoder.
 */

// Temperature calibration (two-wire/10 kOhm v2, 2026-08-17).
//   x = rt_ratio = RT_period / REF_period
//   T [degC] = A*x^2 + B*x + C
//
// A deliberately heated/cooling sweep on the final two-wire/10 kOhm hardware
// produced annotated display points from about 17.6 to 36.6 degC.  A quadratic
// fit is required: the linear one-point re-anchor used on 2026-08-16 leaves
// systematic errors of several degrees across that range.  The quadratic fit
// has about 0.29 degC RMS residual on the current annotated sweep points.
inline constexpr float TEMP_RATIO2_A = 26.151839f;
inline constexpr float TEMP_RATIO_B = -126.906498f;
inline constexpr float TEMP_RATIO_C = 170.744526f;

// Unni LCD display-temperature emulation (2026-08-17).
//   x = RT_period / REF_period
//   T_display [degC] = A*x^2 + B*x + C
//
// This is intentionally separate from the production RT temperature used by
// RH compensation. It is fit to annotated Unni LCD readings, including the
// newly stable low-temperature ~15 degC points around RT/REF ~= 2.29..2.31.
inline constexpr float DISPLAY_TEMP_RATIO2_A = 17.185556f;
inline constexpr float DISPLAY_TEMP_RATIO_B = -94.771485f;
inline constexpr float DISPLAY_TEMP_RATIO_C = 142.237151f;
inline constexpr float DISPLAY_TEMP_RATIO_MIN = 1.54f;
inline constexpr float DISPLAY_TEMP_RATIO_MAX = 2.33f;

// Provisional local-air temperature calibration (2026-08-17).
//
// The external AHT21/BME references do not support a trustworthy fit over the
// full heated sweep. This curve is therefore deliberately constrained to the
// normal-temperature interval actually backed by nearby/same-airflow reference
// points (~18..25 degC). Follow-up cold-air measurements with stronger airflow
// extended the externally checked ratio envelope to about 2.34. Callers should
// publish it only inside the ratio envelope below and retain the last value outside it.
//
//   T_air [degC] = A*x^2 + B*x + C
inline constexpr float AIR_TEMP_RATIO2_A = 48.673890f;
inline constexpr float AIR_TEMP_RATIO_B = -231.233824f;
inline constexpr float AIR_TEMP_RATIO_C = 292.655856f;
inline constexpr float AIR_TEMP_RATIO_MIN = 1.98f;
inline constexpr float AIR_TEMP_RATIO_MAX = 2.35f;

// Relative-humidity calibration (carrier v1, 2026-08-17):
//   r  = RH_carrier_period / REF_period
//   x  = ln(r)
//   RH = A*x^2 + B*x + C*T + D
//
// The previous production decoder used recurrence of the combined RT=0/RH=1
// state. That state disappears when RT and RH become nearly phase-aligned at
// high humidity even though both carriers remain clean. The carrier period is
// directly observable across the tested range and is now the production RH
// quantity.
//
// Initial fit points cover approximately 30..68 %RH and 19..34 degC decoded
// temperature, including both ordinary and deliberately heated captures. This
// is a provisional v1 fit and should be refined with additional stationary
// measurements.
inline constexpr float RH_LOG2_A = 2.666914f;
inline constexpr float RH_LOG_B = -22.589341f;
inline constexpr float RH_TEMP_C = -0.461345f;
inline constexpr float RH_OFFSET = 83.515272f;


// Unni LCD display-humidity emulation (provisional v1, 2026-08-17).
//
// The LCD humidity follows a visibly different scale from the physical carrier
// RH estimate at the cold/high-humidity end. This model is intentionally kept
// separate from RH_LOG2_A/... above so display emulation cannot perturb the
// physical RH path.
//
//   r = RH_carrier_period / REF_period
//   x = ln(r)
//   RH_display = A*x^2 + B*x + C*T_display + D
//
// The initial fit uses annotated LCD points spanning roughly 30..82 %RH. The
// cold stationary points around 80..82 %RH are weighted more strongly than the
// deliberately heated transient points.
inline constexpr float DISPLAY_RH_LOG2_A = 8.119886f;
inline constexpr float DISPLAY_RH_LOG_B = -37.449698f;
inline constexpr float DISPLAY_RH_TEMP_C = -0.579144f;
inline constexpr float DISPLAY_RH_OFFSET = 94.451022f;

// Approximate stationary calibration envelope currently backed by measurements.
// Values outside this envelope are still converted; the caller may expose the
// extrapolation flag diagnostically.
inline constexpr float CAL_TEMP_MIN_C = 17.5f;
inline constexpr float CAL_TEMP_MAX_C = 37.0f;
inline constexpr float CAL_RH_RATIO_MIN = 1.30f;
inline constexpr float CAL_RH_RATIO_MAX = 9.20f;

inline float temperature_from_ratio(float rt_ratio) {
  return TEMP_RATIO2_A * rt_ratio * rt_ratio +
         TEMP_RATIO_B * rt_ratio + TEMP_RATIO_C;
}

inline float display_temperature_from_ratio(float rt_ratio) {
  if (!std::isfinite(rt_ratio))
    return std::numeric_limits<float>::quiet_NaN();
  return DISPLAY_TEMP_RATIO2_A * rt_ratio * rt_ratio +
         DISPLAY_TEMP_RATIO_B * rt_ratio + DISPLAY_TEMP_RATIO_C;
}

inline bool air_temperature_ratio_supported(float rt_ratio) {
  return std::isfinite(rt_ratio) && rt_ratio >= AIR_TEMP_RATIO_MIN &&
         rt_ratio <= AIR_TEMP_RATIO_MAX;
}

inline float air_temperature_from_ratio(float rt_ratio) {
  if (!air_temperature_ratio_supported(rt_ratio))
    return std::numeric_limits<float>::quiet_NaN();
  return AIR_TEMP_RATIO2_A * rt_ratio * rt_ratio +
         AIR_TEMP_RATIO_B * rt_ratio + AIR_TEMP_RATIO_C;
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


inline float display_humidity_from_ratio_temperature(float rh_ratio,
                                                      float display_temperature_c) {
  const float x = log_rh_ratio(rh_ratio);
  if (!std::isfinite(x) || !std::isfinite(display_temperature_c))
    return std::numeric_limits<float>::quiet_NaN();

  float rh = DISPLAY_RH_LOG2_A * x * x + DISPLAY_RH_LOG_B * x +
             DISPLAY_RH_TEMP_C * display_temperature_c + DISPLAY_RH_OFFSET;
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
