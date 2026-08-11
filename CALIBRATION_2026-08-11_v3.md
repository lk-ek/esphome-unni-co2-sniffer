# RT/RH calibration — 2026-08-11 v3

Temperature calibration is unchanged from v2.

Relative humidity now uses a cubic direct fit in:

```cpp
r = rh_state_us / ref_period_us
```

Firmware equation:

```cpp
rh_percent =
      -0.69753285f * r * r * r
      + 10.13034344f * r * r
      - 52.32304359f * r
      + 140.35957354f;
```

This replaces the v2 quadratic, whose minimum fell inside the measured range
and therefore turned upward at low RH.

The cubic fit remains monotonic decreasing over the currently observed
calibration range and matches the added ~42–44 %RH points substantially better.

BLE/history compile-time options and temperature calibration are unchanged.
