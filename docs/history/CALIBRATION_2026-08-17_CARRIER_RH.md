<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# RH carrier calibration v1 — 2026-08-17

The production humidity observable was changed from recurrence of the combined
`RT=0, RH=1` state to the RH-phase carrier period normalized by REF:

```text
r = rh_carrier_period_us / ref_period_us
x = ln(r)
RH = 2.666914*x^2 - 22.589341*x - 0.461345*T + 83.515272
```

## Why

At approximately 60–70 %RH, RT and RH remain electrically active with nearly
matching edge counts, but their phase separation becomes small enough that the
combined `RT=0, RH=1` state can disappear from software GPIO sampling. The old
decoder therefore produced `RH_TOO_FEW_SAMPLES` and no humidity value.

The RH carrier period remains measurable through the same region and also
tracks the legacy RH-state timing closely where both are available. This makes
it a single observable across the tested humidity range.

## Initial calibration data

The first fit uses annotated captures spanning roughly 30..68 %RH, including
deliberately heated transient points. Representative carrier/REF ratios include
about 8.7..9.1 near 30..35 %RH, 4.4..4.8 near 43..44 %RH, 3.48 near 47 %RH,
1.84 near 61 %RH, and about 1.37 near 68 %RH. Temperature compensation remains
part of the model.

The coefficients above were refit after the 2026-08-17 quadratic temperature
calibration replaced the provisional linear RT calibration.  They are fit against
the temperature value that the production decoder now supplies, avoiding a
systematic RH shift caused solely by changing the temperature model.  On the
currently annotated carrier points the refit has about 0.26 %-point RMS residual.

This remains a provisional v1 production calibration. Additional stationary points
should be used for a later refit.

## Validation

RH validity now requires a plausible RH-phase duration, sufficient carrier
cycles, a plausible carrier period, and a plausible carrier/REF ratio. Legacy
RH-state recurrence statistics and direct RT/RH phase measurements remain
diagnostics only.
