<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# v9.1 production power-save

This is the recommended power-save profile after the v9 Automatic Light Sleep
experiment disturbed the timing sniffer and increased CO2 frame errors.

## Kept

- ESP32-C3 CPU: 80 MHz
- Wi-Fi: `power_save_mode: HIGH`
- Wi-Fi: `fast_connect: true`
- BLE advertising interval: 10 s
- BLE live state updated continuously, but the actual advertising payload is
  committed only once per valid RT/RH cycle (~30 s)
- Home Assistant publication: 30 s
- Logger: WARN
- web server retained for timing/capture endpoints
- all v7 quality/refactor and v8 CSV fixes retained

## Removed

- `CONFIG_PM_ENABLE`
- tickless-idle experiment
- `esp_pm_configure()`
- Automatic Light Sleep
- PM lock dump

Reason: with v9, REF timing became less stable, measurement quality sometimes
fell to ~89–95 %, and CO2 frame errors increased continuously. The same behavior
persisted on a floating USB power bank, ruling out protective-earth coupling as
the primary cause.
