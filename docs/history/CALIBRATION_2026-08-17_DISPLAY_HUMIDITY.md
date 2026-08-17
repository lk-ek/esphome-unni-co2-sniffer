<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Unni display humidity emulation — 2026-08-17

The cold/high-humidity measurements established that the Unni LCD humidity and the physical carrier-RH estimate are not the same output scale. The firmware therefore keeps them separate.

`RH Humidity` remains the physical carrier-based estimate and is unchanged by this work. `Unni Display Humidity` is display emulation only.

The provisional v1 LCD model uses:

```text
r = RH_carrier_period / REF_period
x = ln(r)
RH_display = 8.119886*x^2 - 37.449698*x - 0.579144*T_display + 94.451022
```

The initial fit uses annotated LCD points spanning roughly 30..82 %RH. Stationary cold points around 80..82 %RH were weighted more strongly than the deliberately heated transient sweep. Because several mid-range annotations were captured during rapid temperature changes, this model should be treated as provisional display reconstruction rather than a physical humidity calibration.

The physical RH path, its coefficients, BLE humidity and history humidity remain unchanged.
