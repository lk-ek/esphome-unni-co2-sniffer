<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Automatic USB / battery power policy

The ESP32-C3 uses two runtime policies selected automatically from the VBUS detector. USB power favors measurement responsiveness; battery power favors energy efficiency. ESP-IDF automatic Light-sleep is used only as the battery idle mechanism rather than calling `esp_light_sleep_start()` directly.

## USB power mode

While VBUS is present, the component holds persistent `ESP_PM_NO_LIGHT_SLEEP` and `ESP_PM_CPU_FREQ_MAX` locks. The CPU therefore stays awake at 80 MHz, the CO2 decoder captures continuously, and fresh measurements can be forwarded immediately. Removing USB releases those persistent locks and restores the battery policy.

## Battery wake/awake sequence

1. In idle periods ESP-IDF may enter automatic Light-sleep.
2. The configured RT/RH RT or RH GPIO wakes the CPU on the first RT/RH transition (defaults: GPIO3/GPIO4).
3. The RT/RH ISR immediately acquires both an `ESP_PM_NO_LIGHT_SLEEP` lock and an `ESP_PM_CPU_FREQ_MAX` lock.
4. The complete RT/RH waveform is captured without sleeping between edges and the CPU remains fixed at 80 MHz for the active capture window.
5. The decoder finalizes after 100 ms of silence. The measured RT/RH waveform
   itself is approximately 383 ms on the tested Unni hardware.
6. CO2 edge capture is enabled only while this awake window is active. Partial CO2 transactions from idle/sleep intervals are discarded.
7. Both locks remain held until the CO2 decoder sees one valid frame after the RT/RH transaction has completed.
8. Both locks are released; CO2 edge capture is disabled and automatic Light-sleep is permitted again.
9. A configurable failsafe (`light_sleep_max_awake`, default 10 s) releases the locks even if the expected completion path is not reached.

The CO2 SCL/SDA pins are intentionally not GPIO wake sources. Their ISR capture is also disabled outside an active RT/RH awake window. Keeping the CPU awake at 80 MHz after RT/RH until one valid CO2 frame provides a deterministic sample without waking on every CO2 bus transaction during the idle interval or decoding half-frames collected around sleep transitions.

## Configuration

The component owns the required ESP-IDF sdkconfig. Users do **not** need a
`sdkconfig_options:` block for power management or BLE modem sleep. It also
requests the tested 80 MHz ESP32-C3 CPU default, and BLE history requests its
own `senshist` flash partition automatically. When
`light_sleep` is enabled (the default), `co2_monitor_0601/__init__.py` enables power
management, tickless idle and shared PHY/MAC/baseband power-down. BLE builds
also enable Bluetooth controller modem sleep and select the main XTAL as the
Bluetooth low-power clock.

All power-saving defaults remain overrideable from `co2_monitor_0601:` when needed:

```yaml
co2_monitor_0601:
  light_sleep: true             # default
  light_sleep_max_awake: 10s    # default
  ha_publish_interval: 60s              # battery HA throttle, default
  ble_advertising_interval: 2s           # USB default
  ble_battery_advertising_interval: 5s   # battery default
```

The signal GPIOs are also configurable. The tested XIAO ESP32-C3 wiring remains
the default:

```yaml
co2_monitor_0601:
  rt_pin: 3
  rh_pin: 4
  co2_sda_pin: 6
  co2_scl_pin: 7
```

The four GPIOs must be unique. Only the configured RT/RH GPIOs are Light-sleep
wake sources; the configured CO2 SDA/SCL GPIOs are deliberately excluded.

## Important limitations

Automatic Light-sleep is cooperative. Wi-Fi, BLE, timers, or another ESP-IDF
subsystem may hold a power-management lock and reduce or completely prevent
actual Light-sleep residency. The BLE-enabled configurations explicitly enable
ESP-IDF Bluetooth controller modem sleep (Mode 1), select the main XTAL as the
Bluetooth low-power clock, keep it powered when required during Light Sleep,
and permit PHY MAC/baseband power-down. The no-BLE YAML remains the preferred
baseline when measuring the residual BLE cost.

The wake source is level-triggered, not true edge-triggered. The implementation
chooses the wake level opposite the observed idle level of each RT/RH line.

## Publication policy

On USB power, valid CO2 frames are published as they arrive and valid RT/RH measurements are published when each approximately 30-second Unni measurement completes. On battery power, the decoder continues to collect the required measurements but Home Assistant publication is throttled to the latest cached values once per `ha_publish_interval` (60 seconds by default).

BLE advertising switches dynamically between the USB and battery intervals when VBUS changes. The advertising stack is restarted when necessary so the new interval takes effect without rebooting.


## Energy Save Mode test override

The Home Assistant `Energy Save Mode` switch forces the battery power policy while leaving physical USB/VBUS detection unchanged. It is intended for USB power-meter A/B measurements. Enabling it releases the persistent USB no-Light-Sleep/CPU-max locks and uses the same RT/RH-triggered awake windows, CO2 capture gating, BLE battery advertising interval and Home Assistant battery publish throttle as real battery operation. Disabling it immediately restores the normal USB policy if VBUS is physically present.

`energy_save_mode_default` controls the boot state and defaults to `false`. Battery SOC interpretation remains based on physical VBUS, not on the override.


## Energy Save Mode transition

When Energy Save Mode is enabled while VBUS is physically present, `energy_save_grace` (default 3 s) keeps the normal USB PM locks briefly before applying the battery policy. This gives Home Assistant/API and logs time to observe the switch transition before automatic Light-sleep may disconnect native USB Serial/JTAG. Shipped Wi-Fi examples use `power_save_mode: LIGHT`; `HIGH`/MAX_MODEM was unreliable together with automatic Light-sleep on the tested ESP32-C3 setup.

## Wake-window synchronization hardening (2026-08-16)

RT/RH GPIO interrupts acquire the per-cycle `ESP_PM_NO_LIGHT_SLEEP` and
`ESP_PM_CPU_FREQ_MAX` locks. ESP-IDF permits acquire/release from ISR context,
but operations on the same PM-lock handle must not race between ISR and task
context. The wake-window close path therefore masks the two RT/RH GPIO
interrupts while releasing both handles and only marks the window unlocked
after both releases have completed.

Debug builds also keep the current awake window open while raw I2C or RT/RH UDP
packets are queued. This drain grace is bounded to one second beyond the normal
awake timeout, so a missing collector cannot keep a battery-powered device awake
indefinitely.
