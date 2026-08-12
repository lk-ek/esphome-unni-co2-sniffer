# Unni CO₂ Sensor Smartification

This project adds a Seeed Studio XIAO ESP32-C3 to an existing Unni CO₂ monitor as a **passive sniffer**. The original Unni MCU, display, buttons, alarms, CO₂ module, and RT/RH measurement circuitry remain in control; the ESP observes existing signals and publishes decoded measurements.

The firmware exposes:

- CO₂, temperature, and relative humidity through ESPHome/Home Assistant
- Sensirion-compatible BLE live advertisements
- a Sensirion-style GATT history service usable by MyAmbience
- optional raw/debug capture endpoints for reverse engineering

The signal interpretation and calibration were derived experimentally from the tested Unni hardware. Other hardware revisions may require new verification or calibration.

## Features

- passive CO₂ bus capture and frame decoding
- passive RT/RH timing capture using GPIO interrupts
- calibrated temperature and temperature-compensated RH conversion
- measurement-quality, thermal-transient, and calibration-range diagnostics
- native ESPHome API for Home Assistant
- compile-time BLE enable/disable
- Sensirion-compatible live BLE advertising
- persistent Sensirion-style history with GATT download
- compile-time raw capture/debug HTTP support
- production-oriented 80 MHz CPU configuration, Wi-Fi power saving, and optional automatic Light-sleep
- delayed GPIO/ISR initialization for boot isolation
- GPL-3.0-or-later

## Hardware

### Required parts

- Unni CO₂ monitor
- Seeed Studio XIAO ESP32-C3
- fine insulated wire suitable for PCB work
- soldering equipment
- optionally 4.7 kΩ–10 kΩ series resistors for the sniffed signal lines
- a suitable 5 V supply

The XIAO and Unni electronics share **5 V and GND**. The ESP only observes the signal lines.

### Wiring

| XIAO pin | ESP32-C3 GPIO | Unni signal | Function |
|---|---:|---|---|
| D5 | GPIO7 | CO₂ SCL | CO₂ bus clock |
| D4 | GPIO6 | CO₂ SDA | CO₂ bus data |
| D1 | GPIO3 | RT/RH G10 | RT/RH timing |
| D2 | GPIO4 | RT/RH G13 | RT/RH state/timing |
| 5V | — | +5 V | shared supply |
| GND | — | GND | shared ground |

D3 / GPIO5 (Unni G11) is intentionally not connected. A live shadow-decoder test against the former three-line implementation produced identical RH-state median, sample count, and event count, so G11 adds no information used by the production calculation.

The previously investigated G12 signal also duplicates G10 for the purposes of the current decoder and is not required.

For a permanent installation, a series resistor in each ESP sniffing branch is recommended:

```text
Unni signal ---- 10 kΩ ---- XIAO GPIO
```

Do not put the resistor in series with the original Unni signal path.

### Boot isolation and self-heating

The production configuration runs the ESP32-C3 at 80 MHz, uses Wi-Fi power saving, and enables ESP-IDF automatic Light-sleep. `sniffer_start_delay` delays all custom GPIO configuration and ISR attachment; during that period the component leaves the four observed signal pins untouched. The supplied production configurations use 10 seconds.

### Automatic Light-sleep

The supplied YAML files enable `CONFIG_PM_ENABLE` and FreeRTOS tickless idle, then configure:

```yaml
bus_sniffer:
  light_sleep: true
  light_sleep_max_awake: 10s
```

Only the two RT/RH inputs are configured as external Light-sleep wake sources:

- D1 / GPIO3 / G10
- D2 / GPIO4 / G13

The CO₂ bus on GPIO6/GPIO7 is deliberately **not** a wake source. On the first RT/RH edge, the GPIO ISR acquires an ESP-IDF `ESP_PM_NO_LIGHT_SLEEP` lock. This keeps the CPU fully awake while the complete RT/RH transaction is captured. The transaction normally occupies about 383 ms and is finalized after 100 ms of bus silence. The lock then remains held until one subsequent valid CO₂ frame has been decoded. Once both conditions are satisfied the lock is released and automatic Light-sleep may resume. A 10-second failsafe releases the lock if a cycle cannot complete.

Wi-Fi/BLE and other ESP-IDF subsystems can hold their own power-management locks, so `light_sleep: true` permits automatic Light-sleep but does not guarantee that every idle interval reaches Light-sleep. `i2c-sniffer-no-ble.yaml` is the cleanest baseline for measuring the savings attributable to BLE.

This is useful both for boot-isolation testing and because the added ESP sits near temperature-sensitive circuitry.

## Repository layout

```text
.
├── i2c-sniffer.yaml              production build: BLE + history
├── i2c-sniffer-debug.yaml        debug build: BLE + history + web capture
├── i2c-sniffer-no-ble.yaml       genuine no-BLE build
├── bus_sniffer/                  ESPHome external component
│   ├── __init__.py               YAML schema + code generation
│   ├── bus_sniffer.cpp/.h        orchestration, HA publishing, feature wiring
│   ├── co2_decoder.cpp/.h        CO₂ GPIO capture + passive bus decoding
│   ├── rtrh_decoder.cpp/.h       RT/RH ISR capture, quality, calibrated result
│   ├── power_save.cpp/.h         automatic Light-sleep + RT/RH awake-window lock
│   ├── calibration.h             RT/RH calibration model and valid ranges
│   ├── sensirion_sample.h        shared T/RH/CO₂ wire sample representation
│   ├── sensirion_ble.cpp/.h      live BLE advertising and GAP/GATT handling
│   ├── sensirion_history.cpp/.h  RAM/flash history and GATT download
│   └── ble_options.h             compile-time BLE feature flags
├── docs/
│   ├── ISR_ARCHITECTURE.md       current ISR/concurrency design; read before edits
│   └── history/                  dated/superseded engineering notes
└── tools/                        reverse-engineering utilities; not firmware
```

The architecture deliberately keeps timing-critical capture code out of `BusSniffer`. A completed RT/RH cycle is converted into a `rtrh_decoder::Measurement`; the orchestration layer then publishes diagnostics/HA values and updates BLE. Live BLE and history share `SensirionSample`, so the two paths cannot drift through duplicated encoding logic.

## Build variants

### `i2c-sniffer.yaml` — production

This is the normal firmware:

- BLE enabled
- live Sensirion-compatible advertising enabled
- persistent BLE history enabled
- `debug_capture: false`
- `debug_metrics: false`
- logger level `WARN`
- HA publish interval 30 s
- sniffer GPIO initialization delayed by 10 s
- automatic Light-sleep enabled; RT/RH GPIO wake + CO₂ completion gate

The YAML includes the `senshist` data partition required by persistent history.

### `i2c-sniffer-debug.yaml` — capture/debug

This keeps the normal BLE/history features but additionally enables:

- `web_server:`
- `debug_capture: true`
- `debug_metrics: true`
- logger level `DEBUG`

The debug build exposes the capture endpoints implemented by the decoders, including `/capture`, `/rt_rh_capture.csv`, and `/rt_rh_timing.csv`.

Raw capture buffers and their HTTP handlers are compile-time gated and are absent from the normal production binary.

### `i2c-sniffer-no-ble.yaml` — genuine no-BLE

This build contains no `esp32_ble`, no BLE server, no Sensirion history partition, and compiles the component with:

```yaml
ble: false
ble_live: false
ble_history: false
```

It retains the same 80 MHz, Wi-Fi, HA publish, and 10 s sniffer-start behavior as the production build so it is useful for BLE power/timing comparisons.

## ESPHome component configuration

A minimal production-style component block is:

```yaml
bus_sniffer:
  ble: true
  ble_live: true
  ble_history: true
  ble_id: unni_ble
  ble_server_id: unni_ble_server

  ble_advertising_interval: 2s
  ha_publish_interval: 30s
  sniffer_start_delay: 10s
  light_sleep: true
  light_sleep_max_awake: 10s

  debug_capture: false
  debug_metrics: false

  co2:
    name: "CO2"
  rt_temperature:
    name: "RT Temperature"
  rh_humidity:
    name: "RH Humidity"
```

Optional diagnostic outputs are configured independently:

- `crc_errors`, `frame_errors`
- `ref_period`, `rt_period`, `rh_state_period`
- `rt_ratio`, `rh_ratio`, `rh_log`
- `measurement_quality`
- `thermal_transient`
- `temperature_extrapolation`
- `humidity_extrapolation`
- `calibration_extrapolation`

`debug_metrics` controls detailed diagnostic logging. It does not need to be enabled merely to expose the diagnostic entities.

### Compile-time feature flags

The Python component translates YAML feature selection into C++ defines:

- `UNNI_BLE_ENABLED`
- `UNNI_BLE_LIVE_ENABLED`
- `UNNI_BLE_HISTORY_ENABLED`
- `RTRH_DEBUG_CAPTURE`

This is why the no-BLE and no-debug builds actually exclude the respective implementation rather than merely disabling it at runtime.

## Home Assistant behavior

The ESPHome native API is the normal network interface. The primary entities are CO₂, RT-derived temperature, and RH-derived humidity.

`ha_publish_interval` throttles regular Home Assistant updates; the supplied configurations use 30 seconds. Internally the decoders and BLE path can still process new measurements more frequently. Initial valid values are published promptly rather than waiting for the first 30-second interval.

## CO₂ decoder

The CO₂ decoder passively observes SCL/SDA edges and reconstructs traffic without driving the original bus. Completed capture windows are decoded in normal task context rather than in the GPIO ISR. CRC and frame errors can be exposed as ESPHome diagnostic counters.

The CO₂ decoder lives entirely in `bus_sniffer/co2_decoder.*`.

## RT/RH decoder

The RT/RH measurement is not exposed as a simple digital register. The firmware captures timing relationships on G10 and G13 and derives normalized RT/reference and RH/reference values. D3/GPIO5 (the Unni G11 signal used during reverse engineering) is not required by the production decoder.

The RT/RH decoder now finalizes a transaction after 100 ms of silence rather than the historical 15-second quiet delay. This is intentionally longer than the maximum accepted 60 ms RH state period while allowing the power-save state machine to proceed roughly half a second after the RT/RH cycle begins.

The RT/RH decoder owns:

- the edge ISR and phase state
- capture accumulators and snapshot handoff
- period/statistical extraction
- measurement validity/quality
- calibrated temperature and RH result
- calibration extrapolation flags

Calibration coefficients and covered ranges live in `bus_sniffer/calibration.h`.

**Before modifying the GPIO ISR, phase boundaries, capture buffers, or snapshot handoff, read [`docs/ISR_ARCHITECTURE.md`](docs/ISR_ARCHITECTURE.md).** It documents the concurrency assumptions and the code that is timing-critical.

## Sensirion-compatible BLE

The production firmware uses ESPHome's ESP-IDF BLE stack. It does **not** use NimBLE-Arduino or the Sensirion Arduino library at runtime.

The live path emits Sensirion-compatible manufacturer data using the `T_RH_CO2_ALT`-style layout for temperature, RH, and CO₂. The implementation intentionally keeps advertising slow compared with ESPHome defaults to reduce radio activity.

BLE connection/GATT handling is coordinated with ESPHome so a MyAmbience history connection can temporarily own the connection state and normal advertising resumes after disconnect.

## History

With `ble_history: true`, the firmware maintains a Sensirion-style history ring and exposes it through GATT.

Current implementation characteristics include:

- 4096-sample RAM history
- persistent flash journal in the `senshist` partition
- metadata recovery after reboot
- MyAmbience-compatible subscription/download behavior
- shared 8-byte sample encoding with live BLE

Changing the history interval clears/reinitializes stored history as required by the current implementation.

## Building

Create a `secrets.yaml` containing the keys referenced by the chosen YAML, then run for example:

```bash
esphome compile i2c-sniffer.yaml
esphome run i2c-sniffer.yaml
```

For the other variants:

```bash
esphome compile i2c-sniffer-debug.yaml
esphome compile i2c-sniffer-no-ble.yaml
```

## Development documentation

Current documentation:

- [`docs/ISR_ARCHITECTURE.md`](docs/ISR_ARCHITECTURE.md) — ISR, concurrency, capture handoff, timing-sensitive rules
- [`docs/LIGHT_SLEEP.md`](docs/LIGHT_SLEEP.md) — automatic Light-sleep wake/lock state machine and limitations
- [`tools/README.md`](tools/README.md) — reverse-engineering utilities

The files under [`docs/history/`](docs/history/) are intentionally retained as **historical engineering notes**. They record calibration iterations, BLE experiments, power-saving tests, and previous refactors, but may refer to superseded code or configuration. They are not the source of truth for the current architecture.

For current behavior, prefer this README, `docs/ISR_ARCHITECTURE.md`, and the source itself.

## Safety

Opening the monitor and soldering to its electronics can damage the Unni, the ESP, or connected equipment if wired incorrectly. Verify signal levels, pin assignments, polarity, and the power supply on the actual hardware before connecting the XIAO.

The project is not affiliated with or endorsed by the manufacturer of the Unni monitor or by Sensirion. Sensirion names and protocol references describe compatibility with their published gadget ecosystem.

## License

This project is licensed under **GNU General Public License v3.0 or later (`GPL-3.0-or-later`)**. See [LICENSE](LICENSE).
