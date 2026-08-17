<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Temperature calibration v2 — 2026-08-17

The final two-wire RT/RH hookup uses 10 kOhm series resistors between the Unni
measurement points and the XIAO ESP32-C3 inputs.  Calibration data from earlier
direct, three-wire, or four-wire hookups must not be mixed with this hardware.

## Model

The production temperature observable remains:

```text
x = RT_period / REF_period
```

A heated/cooling sweep on 2026-08-17 showed clear curvature that the provisional
linear 2026-08-16 re-anchor could not represent.  Production temperature now uses:

```text
T [degC] = 26.151839*x^2 - 126.906498*x + 170.744526
```

The fit spans annotated Unni display points from approximately 17.6 to 36.6 degC
and has about 0.29 degC RMS residual on the current sweep dataset.

Representative annotated points used for the fit include:

| RT/REF | Unni display |
|---:|---:|
| 1.552988 | 36.6 degC |
| 1.770298 | 28.4 degC |
| 1.829994 | 26.0 degC |
| 1.936817 | 23.4 degC |
| 1.948552 | 22.4 degC |
| 1.990850 | 21.9 degC |
| 2.087128 | 19.2 degC |
| ~2.20..2.25 | ~17.6..18.2 degC |

Several points were deliberately collected during strong thermal transients, so
this is still a field calibration rather than a precision chamber calibration.
Stationary points should be accumulated to validate or refine the curve.

## RH dependency

Carrier-based RH includes decoded temperature as a compensation term.  Therefore
its coefficients were refit together with this temperature change rather than
leaving the previous coefficients tied to the obsolete linear temperature model.
