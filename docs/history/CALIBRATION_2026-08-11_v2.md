# RT/RH calibration — 2026-08-11 v2

Updated with captures 40-20 through 40-22.

Observed reference range now spans approximately:
- 18.1–21.0 °C
- 45–53 %RH

BLE/history compile-time options are unchanged.

## Temperature

```cpp
temperature_c =
    -23.024269f * rt_ratio
    + 67.398734f;
```

RMSE over the available calibration timing captures: about 0.084 °C.

## Relative humidity

The wider data range shows clear curvature. The firmware now uses a quadratic
directly in `rh_ratio = rh_state_us / ref_period_us`:

```cpp
rh_percent =
      1.85955589f * rh_ratio * rh_ratio
    - 20.15071927f * rh_ratio
    + 99.20527595f;
```

RMSE over the available calibration timing captures: about 0.15 %-points RH.

The previous `logf(rh_ratio)` transform is no longer used.
