# Unni CO₂ Sensor Smartification

This project adds a Seeed Studio XIAO ESP32-C3 to an Unni CO₂ monitor as a **passive sniffer**. The original Unni electronics remain in control; the ESP only observes existing signals and publishes the decoded measurements.

## What it provides

- CO₂, temperature and relative humidity in ESPHome / Home Assistant
- Sensirion-compatible BLE advertisements for MyAmbience
- optional persistent BLE history with GATT download
- battery voltage and estimated battery level
- USB/VBUS presence detection
- automatic ESP32-C3 power saving with Light Sleep and BLE modem sleep
- optional diagnostic metrics and raw capture support

The signal decoding and calibration were derived from the tested Unni hardware revision. Other revisions may need verification.

## Hardware

### XIAO ESP32-C3 wiring

| XIAO | GPIO | Connect to | Purpose |
|---|---:|---|---|
| D1 | GPIO3 | Unni **RT** test/sensor point | temperature timing signal |
| D2 | GPIO4 | Unni **RH** test/sensor point | humidity timing signal |
| D4 | GPIO6 | CO₂ SDA | passive CO₂ bus data |
| D5 | GPIO7 | CO₂ SCL | passive CO₂ bus clock |
| D0 | GPIO2 / ADC1_CH2 | midpoint of battery divider | battery voltage |
| D3 | GPIO5 | midpoint of VBUS divider | USB power detection |
| 5V | — | Unni 5 V / VBUS | shared supply |
| GND | — | Unni GND / battery − | common ground |

`RT` and `RH` refer to the PCB points where the unpopulated temperature and humidity sensors can be fitted.

The ESP connections are passive. If desired, add 4.7–10 kΩ series resistors in the ESP sniffing branches:

```text
Unni signal ---- 10 kΩ ---- XIAO GPIO
```

Do not place those resistors in series with the original Unni signal path.

### Battery measurement

The default hardware is:

```text
Battery + --- 1 MΩ ---+--- D0 / GPIO2
                      |
                     1 MΩ
                      |
Battery − / GND ------+

D0 / GPIO2 --- 0.1 µF --- GND
```

The 1 MΩ / 1 MΩ divider gives a ratio of 2.0. The component uses ESP-IDF ADC calibration, 12 dB attenuation and sample averaging.

`Battery Voltage` is always published. `Battery Level` is a voltage-based Li-ion/LiPo estimate and is marked unavailable while USB power is present, because charging voltage is not a useful open-circuit SOC measurement.

### USB/VBUS detection

The tested divider is:

```text
VBUS / 5 V --- 220 kΩ ---+--- D3 / GPIO5
                         |
                        220 kΩ
                         |
                        GND
```

The component publishes this as the `USB Power` binary sensor.

## Default pin assignment

The defaults can be overridden in `bus_sniffer:`:

```yaml
bus_sniffer:
  rt_pin: 3
  rh_pin: 4
  co2_sda_pin: 6
  co2_scl_pin: 7
  battery_pin: 2
  usb_power_pin: 5
```

All configured pins must be unique. `battery_pin` must be an ESP32-C3 ADC1 GPIO (GPIO0–GPIO4).

## ESPHome configuration

The component intentionally owns most platform details. A normal configuration only needs the component itself; all options below already have defaults:

```yaml
bus_sniffer:
  ha_publish_interval: 30s
  sniffer_start_delay: 10s
```

Useful optional settings:

```yaml
bus_sniffer:
  # BLE
  ble: true
  ble_live: true
  ble_history: true
  ble_advertising_interval: 2s

  # Power saving
  light_sleep: true
  light_sleep_max_awake: 10s

  # Hardware
  rt_pin: 3
  rh_pin: 4
  co2_sda_pin: 6
  co2_scl_pin: 7
  battery_pin: 2
  battery_divider_ratio: 2.0
  battery_update_interval: 60s
  usb_power_pin: 5

  # Debugging
  debug_metrics: false
  debug_capture: false
```

The component automatically configures the required ESP-IDF power-management options, BLE server, 80 MHz maximum CPU frequency and BLE-history partition. These do not need to be repeated in user YAML.

## Automatically created entities

Normal builds create:

- `CO2`
- `RT Temperature`
- `RH Humidity`
- `Battery Voltage`
- `Battery Level`
- `USB Power`

The primary sensor definitions can still be overridden, for example:

```yaml
bus_sniffer:
  co2:
    name: "Living Room CO2"
  rt_temperature:
    name: "Temperature"
  rh_humidity:
    name: "Humidity"
```

With `debug_metrics: true`, the component additionally creates decoder-quality, timing, frame-error and calibration diagnostic entities automatically.

## Power saving

Automatic Light Sleep is enabled by default.

The RT and RH GPIOs are Light-Sleep wake sources. On the first RT/RH transition, the component keeps the ESP awake and forces the CPU to 80 MHz until:

1. the RT/RH measurement is complete, and
2. one valid CO₂ frame has been received.

The CO₂ bus itself is not a wake source. Outside an active capture window, partial CO₂ traffic is ignored.

BLE builds also enable ESP32-C3 Bluetooth modem sleep and PHY/MAC/baseband power-down. The default BLE advertising interval is 2 seconds.

## BLE / MyAmbience

BLE is enabled by default. The component advertises a Sensirion-compatible temperature/RH/CO₂ payload and uses the GAP name `S` for MyAmbience compatibility.

With `ble_history: true`, a persistent history ring is stored in the automatically created `senshist` partition and can be downloaded through MyAmbience.

To build without BLE:

```yaml
bus_sniffer:
  ble: false
  ble_live: false
  ble_history: false
```

## Build variants

The repository includes:

- `i2c-sniffer.yaml` — normal build
- `i2c-sniffer-debug.yaml` — debug metrics, web server and raw captures
- `i2c-sniffer-no-ble.yaml` — no-BLE power baseline

Build with ESPHome, for example:

```bash
esphome compile i2c-sniffer.yaml
esphome run i2c-sniffer.yaml
```

## Repository layout

```text
bus_sniffer/          ESPHome external component
  __init__.py         configuration schema and code generation
  bus_sniffer.*       orchestration and entity publishing
  rtrh_decoder.*      RT/RH timing decoder
  co2_decoder.*       passive CO₂ bus decoder
  power_save.*        Light-Sleep capture-window control
  calibration.h       RT/RH calibration
  sensirion_ble.*     BLE live advertising
  sensirion_history.* persistent BLE history / GATT
docs/
  DEVELOPMENT_HISTORY.md development process and design rationale
  ISR_ARCHITECTURE.md    timing-critical ISR design
  LIGHT_SLEEP.md         power-management design
  history/               detailed superseded engineering notes
tools/                reverse-engineering utilities
```

Historical notes under `docs/history/` may use old signal names or describe superseded implementations. The README, active docs and source are the current reference.

## Safety

Verify polarity, signal levels and wiring on the actual hardware before connecting the XIAO. The project is not affiliated with the manufacturer of the Unni monitor or with Sensirion.

## License

GNU General Public License v3.0 or later (`GPL-3.0-or-later`). See [LICENSE](LICENSE).
