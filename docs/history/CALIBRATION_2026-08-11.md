<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# RT/RH calibration refit — 2026-08-11

Basis: synchronized `rt_rh_timing40-*.csv` captures whose filenames encode the original display value.

Usable timing points: **14**. Missing timing CSVs (e.g. 40-11/40-14) were not invented from logs.

Fit used in `co2_monitor_0601.cpp`:

```text
T_C  = -21.433346 * (RT_period / REF_period) + 64.034661
RH_% =  -7.027693 * (RH_state_period / REF_period) + 76.126783
```

Observed fit range: **18.1–19.1 °C** and **50–53 %RH**. Extrapolation outside that range is not yet validated.

In-sample RMSE: **0.091 °C** and **0.175 %-points RH**.

| Capture | Display T | Fit T | ΔT | Display RH | Fit RH | ΔRH |
|---:|---:|---:|---:|---:|---:|---:|
| 40-1 | 18.2 | 18.17 | -0.03 | 53 | 53.07 | +0.07 |
| 40-2 | 18.1 | 18.17 | +0.07 | 53 | 53.07 | +0.07 |
| 40-3 | 18.2 | 18.15 | -0.05 | 53 | 52.89 | -0.11 |
| 40-4 | 18.2 | 18.25 | +0.05 | 53 | 52.89 | -0.11 |
| 40-5 | 18.8 | 18.80 | -0.00 | 51 | 50.96 | -0.04 |
| 40-6 | 18.8 | 18.88 | +0.08 | 51 | 50.99 | -0.01 |
| 40-7 | 18.8 | 18.82 | +0.02 | 51 | 50.87 | -0.13 |
| 40-8 | 19.1 | 19.04 | -0.06 | 50 | 50.41 | +0.41 |
| 40-9 | 18.9 | 19.07 | +0.17 | 50 | 50.32 | +0.32 |
| 40-10 | 19.1 | 18.87 | -0.23 | 50 | 50.04 | +0.04 |
| 40-12 | 18.9 | 18.95 | +0.05 | 50 | 49.97 | -0.03 |
| 40-13 | 19.0 | 18.91 | -0.09 | 50 | 49.95 | -0.05 |
| 40-15 | 18.9 | 18.91 | +0.01 | 50 | 49.77 | -0.23 |
| 40-16 | 18.9 | 18.93 | +0.03 | 50 | 49.79 | -0.21 |

A linear fit was chosen deliberately. On this narrow dataset, quadratic/log-quadratic alternatives reduced RMSE only marginally, so the extra curvature would mainly fit quantization/noise.
