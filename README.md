<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Unni CO₂ Sensor Smartification

This project adds a Seeed Studio XIAO ESP32-C3 to an Unni CO₂ monitor as a **passive sniffer**. The original Unni electronics remain in control; the ESP only observes existing signals and publishes the decoded measurements.

The tested device is sold as **CO2 Monitor Carbon Dioxide Detector 0601**. The ESPHome component is therefore named `co2_monitor_0601`. Older configurations using the former `bus_sniffer:` component key must be changed to `co2_monitor_0601:`.

## What it provides

- CO₂, temperature and relative humidity in ESPHome / Home Assistant
- Sensirion-compatible BLE advertisements for MyAmbience
- optional persistent BLE history with GATT download
- battery voltage and estimated battery level
- USB/VBUS presence detection
- automatic USB/battery power policy with Light Sleep and BLE modem sleep
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

The defaults can be overridden in `co2_monitor_0601:`:

```yaml
co2_monitor_0601:
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
co2_monitor_0601:
  sniffer_start_delay: 10s
```

Useful optional settings:

```yaml
co2_monitor_0601:
  # BLE
  ble: true
  ble_live: true
  ble_history: true
  ble_advertising_interval: 2s          # USB/VBUS power
  ble_battery_advertising_interval: 5s  # battery power

  # Power saving
  light_sleep: true
  light_sleep_max_awake: 10s
  ha_publish_interval: 60s  # battery-mode HA throttle

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
  # Experimental: independent RMT SCL timing assist. Disabled by default.
  rmt_scl_assist: false
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
co2_monitor_0601:
  co2:
    name: "Living Room CO2"
  rt_temperature:
    name: "Temperature"
  rh_humidity:
    name: "Humidity"
```

With `debug_metrics: true`, the component additionally creates decoder-quality, timing, frame-error and calibration diagnostic entities automatically.

In the debug build (`debug_capture: true`, logger level `DEBUG`), structurally valid I²C frames that are not claimed by the CO₂ protocol decoder are logged as `Unhandled I2C frame`. Framing failures are logged separately as `Malformed I2C frame` with a status such as `INCOMPLETE_BYTE`, `CAPTURE_END_IN_FRAME`, or `TRUNCATED`. Malformed frames are never passed to the CO₂ decoder.

The normal GPIO-only path includes two conservative safeguards derived from real VCD captures. First, a combined SDA-setup/SCL-rise sample can be resolved contextually when it occurs inside a byte. Second, when a capture is rejected by the CO₂ protocol decoder, the sniffer may propose insertion of exactly one missing SCL pulse into a suspicious timing gap. That repair is accepted only if exactly one candidate reconstructs a complete `0x62` / `0xEC05` command plus response with the expected ACK/NACK pattern and a valid Sensirion CRC. Timing alone can never make a repair valid.

The first malformed, otherwise unhandled, protocol-invalid, software-recovered, or RMT-repaired CO₂ transaction freezes its **original GPIO** raw edge capture so that later normal traffic cannot overwrite the interesting waveform. A coalesced SDA/SCL sample that is resolved successfully is logged but does not occupy the single freeze slot by itself. Download `/capture` to retrieve the frozen trace; only a successful HTTP transfer releases the freeze, so a failed client can retry. I²C captures observed in practice are small (typically well below 1 KiB) and are sent synchronously by the ESP-IDF HTTP server rather than through a separate FreeRTOS sender task. New captures use the `LA02` format, which preserves the bus state before the first edge. `tools/capture2vcd.py` reads `LA02` and both historical `LA01` header variants.

## Automatic USB / battery power policy

The VBUS detector automatically selects one of two runtime policies. No Home Assistant automation is required.

**USB/VBUS power:**

- automatic Light Sleep is held off
- CPU frequency is held at 80 MHz
- the CO₂ bus is captured continuously
- every valid CO₂ frame is published to Home Assistant immediately
- every valid RT/RH measurement is published immediately
- BLE advertises every 2 seconds by default

**Battery power:**

- the existing RT/RH-triggered automatic Light-Sleep scheme is used
- RT/RH wakes the ESP and opens a short 80 MHz capture window
- CO₂ capture is enabled only until one valid frame has been received after RT/RH
- Home Assistant receives the latest CO₂/T/RH values at most once per 60 seconds
- BLE advertises every 5 seconds by default

The battery HA interval is configured with `ha_publish_interval` (default `60s`). The two BLE intervals are independently configurable with `ble_advertising_interval` (USB, default `2s`) and `ble_battery_advertising_interval` (battery, default `5s`). `light_sleep: false` disables the battery Light-Sleep policy as before.

BLE builds also enable ESP32-C3 Bluetooth modem sleep and PHY/MAC/baseband power-down.

## BLE / MyAmbience

BLE is enabled by default. The component advertises a Sensirion Gadget/MyAmbience-compatible temperature/RH/CO₂ payload and uses the GAP name `S` for protocol compatibility. The BLE Device Information Service identifies the manufacturer as `Gadget`; the firmware does not claim that the device was manufactured by Sensirion.

The production firmware does **not** vendor or directly link the Sensirion Gadget BLE Arduino Library or Sensirion UPT Core. The compatible advertisement, sample encoding, GATT topology and history-download wire format are implemented locally using ESPHome/ESP-IDF BLE APIs. Parts of that compatibility layer were developed with reference to Sensirion Gadget BLE 1.5.0 and Sensirion UPT Core 0.5.1; their BSD-3-Clause notices and detailed provenance are preserved in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and `LICENSES/`.

With `ble_history: true`, a project-specific persistent history ring is stored in the automatically created `senshist` partition and can be downloaded through MyAmbience using the compatible Gadget history protocol.

To build without BLE:

```yaml
co2_monitor_0601:
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
co2_monitor_0601/       ESPHome external component
  __init__.py              configuration schema and code generation
  co2_monitor_0601.*       orchestration and entity publishing
  rtrh_decoder.*           RT/RH timing decoder
  i2c_sniffer.*            generic passive I²C edge capture and framing
  co2_decoder.*            CO₂ protocol/CRC decoder consuming I²C frames
  power_save.*             Light-Sleep capture-window control
  calibration.h            RT/RH calibration
  sensirion_ble.*          BLE live advertising
  sensirion_history.*      persistent BLE history / GATT
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

Project-authored code and documentation are distributed under the GNU General Public License v3.0 or later (`GPL-3.0-or-later`). See [LICENSE](LICENSE).

The Sensirion-derived/referenced BLE compatibility portions retain the applicable upstream BSD-3-Clause notices. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the complete upstream license texts in `LICENSES/`.


## Experimental RMT SCL assist

The normal build uses the GPIO-only I2C capture path. `rmt_scl_assist: false` is
the default and does not link or configure the ESP-IDF RMT driver. This is the
recommended setting after field testing showed Wi-Fi/API instability when RMT
SCL assistance was enabled on the ESP32-C3.

For targeted capture experiments it can be enabled explicitly:

```yaml
co2_monitor_0601:
  rmt_scl_assist: true
```

RMT assistance remains experimental and should be used only for diagnostics. The current experimental implementation reserves 96 RMT symbols so a normal command/response burst fits without the 48-symbol ping-pong copy path, uses interrupt priority 1, and pre-arms `rmt_receive()` from task context. The cache-safe RMT ISR options used by the first experiment are no longer forced. The GPIO decoder and protocol-validated single-clock recovery remain available without RMT.
