<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# v6 calibration refactor

The timing decoder and calibration model are now separated.

- `co2_monitor_0601.cpp`: capture, phase decoding, quality checks, diagnostics and publishing
- `co2_monitor_0601/calibration.h`: calibration coefficients, model functions and calibration envelope
- BLE/history modules remain unchanged

The v4 temperature and RH coefficients are unchanged. This is a structural refactor only.

Decoder-quality rules such as the 32-sample RH minimum remain in `co2_monitor_0601.cpp`; they describe capture validity rather than the sensor calibration.
