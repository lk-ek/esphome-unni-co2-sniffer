<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Dual temperature views — 2026-08-17

The stable cold-air measurements showed that one RT/REF conversion cannot simultaneously emulate the Unni LCD and agree with an external sensor positioned immediately beside the open Unni sensor area. The firmware therefore keeps these goals separate.

## RT Temperature

`RT Temperature` remains the existing v2 decoder temperature. It is intentionally left unchanged because carrier-RH v1 was fitted using this value as its temperature-compensation input. It is now treated as a diagnostic/raw-model view rather than the preferred exported air temperature.

## Unni Display Temperature

The LCD-emulation observable is still `x = RT_period / REF_period` and uses:

```text
T_display = 17.185556*x^2 - 94.771485*x + 142.237151
```

The fit combines the 2026-08-17 annotated heated/cooling display sweep with the later stationary low-temperature points around `RT/REF ~= 2.29..2.31`, where the LCD showed about 15.0 °C. The fit RMS on the currently annotated display points is about 0.31 °C.

## Air Temperature

The first physical-air curve is deliberately limited to the normal-temperature region actually supported by nearby/same-airflow AHT21/BME280 observations:

```text
T_air = 48.673890*x^2 - 231.233824*x + 292.655856
```

Supported ratio envelope:

```text
1.98 <= RT/REF <= 2.35
```

Representative reference anchors include the earlier same-airflow ~25.15 °C AHT21 point, the ~19.5 °C nearby-reference point, and the 2026-08-17 cold stationary point with the AHT21 positioned about 2 cm from the Unni sensor area (~18.38 °C while RT/REF was about 2.29). A later high-airflow cold run provided additional external checks through roughly `RT/REF ~= 2.34`, with `Air Temperature` remaining in the AHT21/BME280 range, so the supported upper bound was extended to 2.35 without refitting the curve. Because the external references did not accompany the full heat-gun sweep, the air curve must not be treated as calibrated outside this envelope.

## Publication policy

`Air Temperature` is published only when its ratio is inside the supported envelope. Outside it, the last supported Home Assistant value is retained. Sensirion BLE live advertisements and persistent history use `Air Temperature` when available, falling back to `RT Temperature` only outside the externally validated air-temperature envelope. `RT Temperature` continues to drive carrier-RH temperature compensation so this change does not silently refit physical humidity. `Unni Display Temperature` remains display emulation only.
