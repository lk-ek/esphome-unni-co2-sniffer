<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# RT temperature calibration — 2026-08-16 provisional two-wire / 10 kOhm refit

The RT/RH pickup hardware changed after the 2026-08-11 calibration dataset:

- only the two currently required RT/RH measurement lines remain connected;
- each measurement line now has a 10 kOhm series resistor between the Unni PCB
  test point and the ESP32-C3.

The old timing captures therefore no longer describe the electrical loading of
the current hardware closely enough to be used as direct calibration points.

Three stationary captures taken with the current hardware produced:

| sequence | REF [us] | RT [us] | RT / REF |
|---:|---:|---:|---:|
| 6 | 77.704 | 154.384 | 1.986822 |
| 7 | 77.529 | 154.498 | 1.992777 |
| 8 | 77.672 | 155.260 | 1.998919 |

The Unni display read 24.0 °C.  The mean normalized ratio is 1.992839.
Nearby BME280 and AHT21 sensors, placed in the same air-filter flow, read
25.0 °C and 25.15 °C respectively.  Those external sensors are retained as
context only: this decoder calibration is intentionally anchored to the Unni
display value.

With only one new display-temperature anchor, the previous fitted slope is kept
unchanged and only the intercept is shifted:

```cpp
T = -23.024269 * rt_ratio + 69.883663;
```

This makes `rt_ratio = 1.992839` evaluate to 24.0 °C.

The change is provisional.  Once a temperature sweep with the current hardware
is available, both slope and intercept (and, if warranted, model shape) should
be fitted again from the new dataset.

The RH coefficients are intentionally unchanged in this revision.  Although
the current RH estimate differs from the Unni display, a single stationary RH
point is insufficient to refit the temperature-compensated RH model safely.
