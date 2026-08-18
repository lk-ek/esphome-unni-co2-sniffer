# Current-hardware RT/RH regression corpus (2026-08-17/18)

Source archive: `unni-captures-20260817-141938-135658.zip`.

This corpus represents the current two-wire RT/RH hardware with 10 kΩ series resistors and the carrier-based RH decoder. The raw archive remains the source material; the CSV fixture is a compact deterministic selection for native host regression tests.

## Source capture summary

- Batch interval: `2026-08-17T14:19:48.123+02:00` to `2026-08-18T13:56:47.813+02:00`
- Timing records: **2227**
- Valid measurements: **1171**
- Rejected measurements: **1056**
- Raw RT/RH CSV captures in archive: **2190**
- I²C CSV captures in archive: **331**

Rejected timing records:

- `REF_PERIOD` (1): **131**
- `RH_DURATION` (6): **925**

Valid-measurement envelope:

| Quantity | Min | Median | Max |
|---|---:|---:|---:|
| `quality_percent` | 69.886 | 91.058 | 95.688 |
| `ref_period_us` | 76.814 | 77.082 | 79.015 |
| `rt_period_us` | 162.374 | 169.351 | 178.523 |
| `rh_carrier_period_us` | 88.337 | 103.048 | 131.464 |
| `rh_carrier_ref_ratio` | 1.129 | 1.336 | 1.706 |
| `rh_carrier_count` | 975.000 | 1261.000 | 1483.000 |
| `temperature_c` | 17.107 | 18.198 | 19.988 |
| `humidity_percent` | 63.370 | 68.776 | 72.597 |

## Fixture selection

`rtrh_current_260817.csv` contains **256** valid measurements selected deterministically from the 1171 valid timing records.

The selection combines evenly spaced samples across the full ~24 h batch with nearest samples to 0/1/5/10/25/50/75/90/95/99/100% quantiles of REF period, RT period, RH carrier period, RH/REF ratio, quality, decoded temperature, and decoded humidity. This preserves temporal coverage and the observed extrema without embedding thousands of raw edge files in the test suite.

The golden engineering-unit columns are computed with the production calibration coefficients at corpus creation time and are intentionally frozen. A later calibration refit must therefore update this fixture explicitly.

Important: this dataset is a **regression corpus**, not independent calibration truth. The decoded temperature/humidity values in the source timing CSV were produced by the firmware itself. External-reference calibration points should remain separate.
