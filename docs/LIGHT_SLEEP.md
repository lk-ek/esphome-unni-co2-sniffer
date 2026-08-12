# Automatic Light-sleep experiment

The ESP32-C3 build uses ESP-IDF automatic Light-sleep rather than calling
`esp_light_sleep_start()` directly. This is important because the node still
runs ESPHome networking and, in the normal build, BLE.

## Wake/awake sequence

1. In idle periods ESP-IDF may enter automatic Light-sleep.
2. GPIO3/G10 or GPIO4/G13 wakes the CPU on the first RT/RH transition.
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

The ESP-IDF build needs power management and tickless idle:

```yaml
esp32:
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_PM_ENABLE: y
      CONFIG_FREERTOS_USE_TICKLESS_IDLE: y
      CONFIG_BT_CTRL_MODEM_SLEEP: y
      CONFIG_BT_CTRL_MODEM_SLEEP_MODE_1: y
      CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL: y
      CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP: y
      CONFIG_ESP_PHY_MAC_BB_PD: y
```

The component options are:

```yaml
bus_sniffer:
  light_sleep: true
  light_sleep_max_awake: 10s
```

When enabled the component configures automatic power management for 40..80 MHz
and enables GPIO wake for GPIO3/GPIO4 on the level opposite the idle level seen
at initialization.

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
