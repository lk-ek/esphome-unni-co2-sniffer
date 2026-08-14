<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# RT/RH calibration — 2026-08-11 v4

Temperature calibration is unchanged.

Relative humidity now uses a temperature-compensated log-quadratic model:

```cpp
r = rh_state_us / ref_period_us;
x = logf(r);

RH =
      6.11947870 * x*x
    - 33.93748066 * x
    - 0.48564674 * temperature_c
    + 93.38516444;
```

The coefficients were fitted from the 30 currently available timing calibration
files using the display values encoded in the filenames.

RMSE against those quantized display values is about 0.32 %-points RH when
evaluated with the firmware temperature estimate.

The fitted temperature coefficient is about -0.486 %-points RH / degree C,
which is physically plausible for common resistive polymer humidity sensors.

This is intentionally an intermediate calibration. It should be refined after
the remaining resistor-network values and exact humidity sensor type are known.
