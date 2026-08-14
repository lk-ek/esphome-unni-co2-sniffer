<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Refactor v11 — shared Sensirion sample + smaller BLE live layer

This revision keeps the working ESP-IDF/ESPHome BLE implementation and reduces
its custom code without switching frameworks.

## Changes

- Added `sensirion_sample.h` as the single data/encoding model for T/RH/CO2.
- `sensirion_ble.cpp` reduced from 561 to ~263 lines.
- Replaced the dynamic advertisement `std::vector` with a fixed 26-byte array.
- Removed the public temperature/humidity/CO2 getter set and duplicate encoder
  API. History reads one `SensirionSample` instead.
- History now uses the same 8-byte encoded sample as live advertising.
- BLE sample state is updated whenever BLE is enabled; live advertisement
  commit remains guarded by `ble_live`. This also makes `ble_history: true` +
  `ble_live: false` internally coherent.
- Preserved the working custom ESP-IDF GAP state machine, slow advertising
  interval, GATT connection protection, post-disconnect advertiser takeover,
  test BT identity, MyAmbience payload, flash history and history download.

## Intentionally not changed

- History flash journal/layout and metadata format.
- GATT UUIDs / characteristic semantics.
- History download packet format and timing.
- Sensor decoding, calibration, quality, HA publishing.

## Size

Component C++/headers are ~2705 lines vs ~3363 in v10/v3.
