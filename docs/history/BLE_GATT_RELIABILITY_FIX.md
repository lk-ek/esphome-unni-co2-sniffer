<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# BLE / MyAmbience GATT reliability fix

Changes:

- `ble_advertising_interval` reduced from 10 s to 2 s.  Ten seconds is too slow
  for reliable interactive discovery/connection in MyAmbience while still being
  unnecessary for the desired power saving.
- The custom advertising state machine now tracks `ESP_GATTS_CONNECT_EVT` and
  `ESP_GATTS_DISCONNECT_EVT`.
- While a GATT client is connected, live T/RH updates only replace the pending
  manufacturer payload in RAM. They never call `stop_advertising`,
  `config_adv_data_raw`, or `start_advertising`.
- GAP completion callbacks are connection-aware, closing races where an async
  advertising configuration completed just after a GATT connection was accepted.
- On disconnect the current Sensirion payload is restored using the configured
  low-duty advertising interval.

Expected log lines during a MyAmbience connection:

```
Sensirion GATT connected (conn=...); advertising refresh paused
... history GATT traffic ...
Sensirion GATT disconnected; restoring slow advertising
```

The 10 s boot GPIO isolation test from the previous build is retained.
Calibration coefficients are unchanged.
