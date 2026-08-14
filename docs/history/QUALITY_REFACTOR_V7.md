<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# v7 quality refactor

Calibration coefficients are unchanged from v4/v6.

## Structure

- `calibration.h`: sensor calibration model and validated calibration envelope
- `measurement_quality.h`: decoder acceptance rules, score and structured reject reason
- `co2_monitor_0601.cpp`: capture/decoder, diagnostics, publishing, BLE/history integration

## Structured reject reasons

Rejected RT/RH measurements now report one deterministic reason:

- `REF_PERIOD`
- `REF_DURATION`
- `RT_PERIOD`
- `RT_DURATION`
- `RT_COUNT`
- `RH_DURATION`
- `RH_TOO_FEW_SAMPLES`
- `RH_STATE_PERIOD`
- `RH_RATIO_IMPLAUSIBLE`

The timing CSV contains the reject reason as well.

## Thermal transient

Transient detection now uses absolute temperature rate rather than raw
temperature delta:

- ON at >= 0.8 C/min by default
- OFF at <= 0.3 C/min by default

This adds hysteresis and remains meaningful if the measurement interval changes.

## Calibration extrapolation

Three diagnostic flags are available:

- temperature extrapolation
- humidity-ratio extrapolation
- combined calibration extrapolation

The current calculation is still published when extrapolating.

## Measurement quality

The score combines:

- REF-period closeness
- RT cycle fill
- RH recurrence sample fill
- RH observation/sample density

Acceptance/rejection is separate from the score.
