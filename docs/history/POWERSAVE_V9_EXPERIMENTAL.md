<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# v9 experimental power save / Automatic Light Sleep

Built on v8 powersave + CSV fix. Calibration and quality logic are unchanged.

## Common changes

- CPU fixed at 80 MHz
- Wi-Fi `power_save_mode: HIGH`
- Wi-Fi `fast_connect: true`
- HA publish interval 30 s
- BLE live payload committed once per RT/RH cycle
- BLE advertising interval 10 s
- logger level WARN
- web server intentionally retained because bus_sniffer capture/timing HTTP
  handlers depend on ESPHome's web server infrastructure

## ESP-IDF power management

The ESP-IDF build enables:

- `CONFIG_PM_ENABLE=y`
- `CONFIG_FREERTOS_USE_TICKLESS_IDLE=y`

`bus_sniffer` then calls:

- max CPU: 80 MHz
- min CPU: 80 MHz
- automatic Light Sleep: enabled

This deliberately tests Light Sleep without DFS.

## Important BLE limitation

ESP-IDF's Bluetooth controller holds an `ESP_PM_NO_LIGHT_SLEEP` lock while
Bluetooth is enabled, including Bluetooth Modem-sleep. Therefore:

- `i2c-sniffer.yaml`: PM is enabled, but active BLE is expected to prevent
  actual automatic Light Sleep. It still benefits from Wi-Fi modem sleep,
  80 MHz CPU, 10 s BLE advertising, and reduced BLE payload reconfiguration.
- `i2c-sniffer-no-ble.yaml`: genuine A/B profile where automatic Light Sleep
  can actually occur when the system is idle.

At startup v9 calls `esp_pm_dump_locks(stdout)` once. This is intentional:
compare the BLE and no-BLE builds and look for `NO_LIGHT_SLEEP` locks.

## Test plan

1. Flash `i2c-sniffer.yaml`, measure steady current and spikes.
2. Record RT/RH quality and CO2 Frame Errors.
3. Flash `i2c-sniffer-no-ble.yaml`, repeat measurement.
4. The difference isolates the BLE controller/advertising cost and shows how
   much Automatic Light Sleep can save on this workload.

If the no-BLE Light-Sleep build causes capture/reject problems, disable
`experimental_light_sleep` before changing the 80 MHz CPU setting.
