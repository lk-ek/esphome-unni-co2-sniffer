# Automatic Light-sleep experiment

The ESP32-C3 build uses ESP-IDF automatic Light-sleep rather than calling
`esp_light_sleep_start()` directly. This is important because the node still
runs ESPHome networking and, in the normal build, BLE.

## Wake/awake sequence

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
`light_sleep` is enabled (the default), `bus_sniffer/__init__.py` enables power
management, tickless idle and shared PHY/MAC/baseband power-down. BLE builds
also enable Bluetooth controller modem sleep and select the main XTAL as the
Bluetooth low-power clock.

All power-saving defaults remain overrideable from `bus_sniffer:` when needed:

```yaml
bus_sniffer:
  light_sleep: true             # default
  light_sleep_max_awake: 10s    # default
  ble_advertising_interval: 2s  # default
```

The signal GPIOs are also configurable. The tested XIAO ESP32-C3 wiring remains
the default:

```yaml
bus_sniffer:
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
