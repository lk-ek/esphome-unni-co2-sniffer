# RT/RH decoder v5 — diagnostics and quality

Built on the temperature-compensated v4 calibration. The calibration
coefficients themselves are unchanged.

## Breath-test finding

During the breath test, measurement 52 reported a median RH-state recurrence
of 6689.5 us with only 12 samples / 13 observations. The old quality check
accepted any >=8 samples, so this aliased cadence was incorrectly published.

v5 requires at least 32 RH recurrence samples and also rejects RH/REF ratios
above 20. Normal captures typically fill all 96 recurrence slots.

## New diagnostics

Optional Home Assistant diagnostic entities expose:

- REF period
- RT period
- RH state period
- RT/REF ratio
- RH-state/REF ratio
- ln(RH-state/REF)
- measurement quality (0–100 %)
- thermal transient flag
- calibration extrapolation flag

`debug_metrics: true` also adds one compact diagnostic log line per valid RT/RH
measurement.

## Thermal transient

A valid measurement is marked transient when temperature changes by at least
`thermal_transient_threshold` since the previous valid measurement. Default:
0.4 C.

This is informational: the measurement is still published.

## Calibration extrapolation

The current well-validated stationary envelope is approximately:

- 18–23 C
- RH/REF ratio 3.20–8.20

Measurements outside the envelope are still calculated and published, but the
diagnostic flag is set.

## CSV metadata

`/rt_rh_timing.csv` now includes derived values on the RH row:

- valid
- rt_ratio
- rh_ratio
- temperature_c
- humidity_percent
- quality_percent
- thermal_transient
- calibration_extrapolation

This removes most of the manual filename/log correlation needed during future
calibration sessions.
