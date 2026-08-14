<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Boot GPIO isolation test

The custom sniffer now supports `sniffer_start_delay`.

The production/test YAML sets:

```yaml
bus_sniffer:
  sniffer_start_delay: 10s
```

During those first 10 seconds the custom component does not call `gpio_config`,
`gpio_set_intr_type`, `gpio_get_level`, or install ISR handlers for the five
Unni signal lines (XIAO D1/D2/D3/D5/D6; GPIO3/4/5/6/7). Wi-Fi/BLE/history are
allowed to initialize normally. After the delay all five lines are configured as
plain inputs with both internal pull resistors disabled and the edge ISRs are
attached.

Set `sniffer_start_delay: 0s` to restore the previous behavior.

Calibration coefficients were intentionally left unchanged. The post-boot BME280
reference series indicates that the existing temperature decoding is already much
closer to local air temperature than the transient Unni display value, while the
remaining RH difference is not yet sufficiently constrained for a safe curve refit.
