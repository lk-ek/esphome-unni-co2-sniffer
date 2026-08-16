<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Unni CO₂ Sensor Smartification

ESPHome firmware for adding a Seeed Studio XIAO ESP32-C3 to an Unni CO₂ monitor without taking control away from the original electronics.

The ESP32-C3 acts primarily as a **passive sniffer**:

- CO₂ is decoded from the existing I²C traffic between the Unni controller and the CO₂ module.
- Temperature and relative humidity are reconstructed from the existing RT/RH timing signals.
- Measurements are exposed to Home Assistant through ESPHome.
- The same measurements can be advertised over BLE in a Sensirion MyAmbience-compatible format.
- Optional persistent BLE history allows historical samples to be downloaded with MyAmbience.

The tested monitor is sold as **CO2 Monitor Carbon Dioxide Detector 0601**, hence the ESPHome component name:

```yaml
co2_monitor_0601:
```

This project was developed against one specific hardware revision. Other revisions should be verified before connecting the ESP.

---

## Features

### Measurements

- CO₂ concentration
- temperature
- relative humidity
- battery voltage
- estimated battery state of charge
- USB/VBUS presence

### Integrations

- ESPHome / Home Assistant
- Sensirion-compatible BLE live advertisements
- persistent BLE history with MyAmbience-compatible GATT download

### Power management

- automatic distinction between USB and battery operation
- ESP32-C3 dynamic frequency scaling
- automatic Light Sleep in battery mode
- reduced BLE advertising rate on battery
- reduced Home Assistant publication rate on battery
- optional `Energy Save Mode` for measuring battery-style firmware behavior while physically powered through USB
- dedicated BLE-only measurement build without Wi-Fi or Home Assistant

### Diagnostics

Optional debug builds provide:

- RT/RH timing diagnostics
- I²C framing and decoder diagnostics
- raw I²C captures
- VCD conversion tools
- recovery diagnostics for occasionally missed GPIO edges

---

# Hardware

## XIAO ESP32-C3 wiring

| XIAO | GPIO | Unni connection | Purpose |
|---|---:|---|---|
| D1 | GPIO3 | RT | temperature timing signal |
| D2 | GPIO4 | RH | humidity timing signal |
| D4 | GPIO6 | CO₂ SDA | passive I²C data sniffing |
| D5 | GPIO7 | CO₂ SCL | passive I²C clock sniffing |
| D0 | GPIO2 / ADC1_CH2 | battery-divider midpoint | battery voltage |
| D3 | GPIO5 | VBUS-divider midpoint | USB power detection |
| 5V | — | Unni VBUS / 5 V | ESP supply |
| GND | — | Unni GND / battery − | common ground |

`RT` and `RH` are the PCB points associated with the temperature and humidity measurement circuitry.

The ESP must remain electrically passive with respect to the original signal paths.

Optional series resistors can be added to the ESP branches:

```text
Unni signal ---- 4.7–10 kΩ ---- XIAO GPIO
```

Do not put these resistors in series with the signal path used by the original Unni controller.

---

## Battery voltage measurement

The tested circuit uses two 1 MΩ resistors:

```text
Battery + --- 1 MΩ ---+--- D0 / GPIO2
                      |
                     1 MΩ
                      |
Battery − / GND ------+

D0 / GPIO2 --- 0.1 µF --- GND
```

This produces a divider ratio of 2.0.

The firmware uses ESP-IDF ADC calibration, averaging and 12 dB attenuation.

Battery-related entities include:

- `Battery Voltage`
- `Battery Level`
- `Battery Runtime Estimate`
- `Battery Charge Time Estimate`
- `Battery Discharge Rate` (`%/h`, diagnostic when `debug_metrics: true`)
- `Battery Charge Rate` (`%/h`, diagnostic when `debug_metrics: true`)

`Battery Level` is a voltage-based Li-ion estimate. It is intentionally unavailable while physical USB power is present because charger-driven battery voltage is not a useful open-circuit state-of-charge measurement.

Debug builds intentionally limit the native ESPHome API to one simultaneous client while raw UDP capture is enabled. Home Assistant should keep that slot; use USB serial for live `esphome logs`. Additional native API log clients can consume enough heap on the ESP32-C3 to starve lwIP. If UDP export sees sustained `ENOMEM` pressure, the affected debug capture is dropped after repeated failures instead of retrying indefinitely and holding memory/network resources.

The runtime and charge-time estimates are learned from the observed voltage trend rather than from a fixed assumed current draw. Each USB/battery transition starts a new learning session, followed by a two-minute settling period. The estimator then uses at least a five-minute observation window and exponentially smooths subsequent rate measurements. Until a meaningful trend is available, the corresponding estimate remains unavailable instead of publishing a speculative value.

On battery, the estimator uses the normal voltage-derived SOC curve. With USB present, `Battery Level` stays unavailable and the charge estimator uses the charger-influenced battery-node voltage only as a charge-progress proxy. The charge ETA is therefore inherently less accurate, especially in the constant-voltage/taper region near full charge.

---

## USB/VBUS detection

The tested VBUS divider is:

```text
VBUS / 5 V --- 220 kΩ ---+--- D3 / GPIO5
                         |
                        220 kΩ
                         |
                        GND
```

The physical state is exposed as:

```text
USB Power
```

The reported value always reflects actual VBUS presence, even when the firmware is deliberately using the battery power policy through `Energy Save Mode`.

---

# Installation

A minimal normal configuration is:

```yaml
esphome:
  name: i2csniffer

external_components:
  - source:
      type: local
      path: .

esp32:
  board: seeed_xiao_esp32c3
  framework:
    type: esp-idf

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
  encryption:
    key: !secret i2csniffer__encryption_key

co2_monitor_0601:
```

The repository contains complete example configurations, so in normal use it is easier to start with one of those rather than build the YAML from scratch.

Build or flash with ESPHome:

```bash
esphome compile i2c-sniffer.yaml
esphome run i2c-sniffer.yaml
```

---

# Build variants

The repository contains several configurations for different purposes.

| Configuration | Wi-Fi / HA | BLE | Debug capture | Purpose |
|---|---|---|---|---|
| `i2c-sniffer.yaml` | yes | yes | no | normal operation |
| `i2c-sniffer-debug.yaml` | yes | yes | yes | protocol/debugging work |
| `i2c-sniffer-no-ble.yaml` | yes | no | optional | BLE power comparison |
| `i2c-sniffer-ble-only.yaml` | no | yes | no | BLE-only power measurement |
| `i2c-sniffer-sht43-probe.yaml` | yes | experimental | yes | MyAmbience reverse engineering |

The SHT43 probe is diagnostic firmware and should not be used as the normal device identity.

---

# Home Assistant

Home Assistant support is enabled by default:

```yaml
co2_monitor_0601:
  home_assistant: true
```

Omitting `home_assistant` is equivalent to setting it to `true`.

Only the dedicated BLE-only build intentionally uses:

```yaml
co2_monitor_0601:
  home_assistant: false
```

In that build the local ESPHome entity objects remain registered internally so ESPHome 2026.8 can derive its entity-vector sizes correctly, but the supplied YAML contains no `wifi:` or `api:` component, so they are not exposed to Home Assistant and cause no network traffic.

Normal builds create:

- `CO2`
- `RT Temperature`
- `RH Humidity`
- `Battery Voltage`
- `Battery Level`
- `Battery Runtime Estimate`
- `Battery Charge Time Estimate`
- `USB Power`
- `Energy Save Mode`
- `BLE Pairing Mode` while the experimental secure MyAmbience settings support is enabled

Sensor definitions may be customized:

```yaml
co2_monitor_0601:
  co2:
    name: "Living Room CO2"

  rt_temperature:
    name: "Temperature"

  rh_humidity:
    name: "Humidity"
```

With `debug_metrics: true`, additional timing, quality and decoder diagnostic entities are created.

---

# Component configuration

Useful options include:

```yaml
co2_monitor_0601:
  # Home Assistant
  home_assistant: true
  ha_publish_interval: 60s

  # BLE
  ble: true
  ble_live: true
  ble_history: true
  ble_advertising_interval: 2s
  ble_battery_advertising_interval: 5s

  # Power management
  light_sleep: true
  light_sleep_max_awake: 10s
  energy_save_mode_default: false
  energy_save_grace: 3s

  # Hardware
  rt_pin: 3
  rh_pin: 4
  co2_sda_pin: 6
  co2_scl_pin: 7
  battery_pin: 2
  battery_divider_ratio: 2.0
  battery_update_interval: 60s
  usb_power_pin: 5

  # Startup
  sniffer_start_delay: 10s

  # Diagnostics
  debug_metrics: false
  debug_capture: false
```

The defaults match the tested XIAO ESP32-C3 installation.

All configured GPIOs must be unique.

`battery_pin` must be an ESP32-C3 ADC1-capable GPIO.

The component configures the ESP-IDF power-management requirements, BLE server and persistent history partition automatically.

---

# USB and battery power policy

The firmware uses different runtime policies depending on physical power.

## USB power

Normal USB operation prioritizes responsiveness and reliability:

- automatic MCU Light Sleep is prevented
- CPU frequency is held at 80 MHz
- Wi-Fi power saving is disabled at runtime with `WIFI_PS_NONE`
- CO₂ I²C capture remains continuously enabled
- valid CO₂ measurements are published immediately
- valid RT/RH measurements are published immediately
- BLE advertisements use the USB interval, default `2s`

Because USB power is available, reducing ESP consumption is not the priority in this mode.

## Battery power

Battery operation prioritizes low average power:

- automatic Light Sleep is enabled
- GPIO3/GPIO4 RT/RH activity wakes the ESP
- CPU frequency is temporarily raised to 80 MHz while measurements are captured
- CO₂ sniffing is enabled only for the active measurement window
- after RT/RH and a valid CO₂ sample are obtained, the ESP may sleep again
- Wi-Fi uses `WIFI_PS_MIN_MODEM`
- Home Assistant publication is throttled to the latest values, default once per minute
- BLE advertisements use the battery interval, default `5s`

The corresponding options are:

```yaml
co2_monitor_0601:
  light_sleep: true
  light_sleep_max_awake: 10s
  ha_publish_interval: 60s
  ble_advertising_interval: 2s
  ble_battery_advertising_interval: 5s
```

---

# Energy Save Mode

`Energy Save Mode` exists mainly for controlled power measurements.

When enabled while the hardware is still physically powered from USB, the component behaves as if USB power were absent.

This allows a USB power meter to compare normal firmware behavior against the battery-oriented runtime policy without changing the electrical power source.

The physical `USB Power` entity remains truthful.

In Energy Save Mode:

- automatic Light Sleep is enabled
- the battery capture-window policy is used
- Wi-Fi uses `WIFI_PS_MIN_MODEM`
- BLE uses the battery advertising interval
- Home Assistant publication uses the battery throttle

The default is:

```yaml
co2_monitor_0601:
  energy_save_mode_default: false
  energy_save_grace: 3s
```

The grace period keeps the normal USB power locks briefly after the HA switch is enabled so that the state change can propagate before automatic Light Sleep becomes active.

Native USB Serial/JTAG may become unavailable while automatic Light Sleep is active. This does not affect the ESP32-C3 ROM USB bootloader, so USB flashing remains possible.

---

# BLE-only measurement build

`i2c-sniffer-ble-only.yaml` is intended for measuring the cost of BLE without Wi-Fi/API overhead.

It intentionally contains no:

- `wifi:`
- `api:`
- captive portal
- OTA

It keeps:

- sensor decoding
- BLE live data
- BLE history
- power management

The relevant configuration is:

```yaml
co2_monitor_0601:
  home_assistant: false

  ble: true
  ble_live: true
  ble_history: true

  energy_save_mode_default: true
  energy_save_grace: 0s
```

Flash this build over USB.

Recent example measurements on the tested hardware were approximately:

| Mode | Average power |
|---|---:|
| normal Wi-Fi + HA + BLE | 2.84 mW |
| Energy Save Mode | 2.07 mW |
| BLE-only | 1.34 mW |

These measurements were made with a USB power meter and should be treated as comparative rather than laboratory-grade absolute values.

Avoid running `ping`, `esphome logs`, an actively polling web UI or other unnecessary network traffic during power measurements.

---

# Sensirion / MyAmbience compatibility

The normal firmware advertises temperature, humidity and CO₂ in a Sensirion Gadget/MyAmbience-compatible format.

The GAP name used by the normal compatibility mode is:

```text
S
```

The Device Information Service identifies the manufacturer as:

```text
Gadget
```

The firmware does not claim that the hardware was manufactured by Sensirion.

The BLE compatibility implementation is local code using ESPHome and ESP-IDF BLE APIs. The production firmware does not directly vendor or link the Sensirion Gadget BLE Arduino Library or Sensirion UPT Core.

Parts of the protocol implementation were developed with reference to Sensirion's published software. See:

- [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
- `LICENSES/`

for attribution, provenance and upstream license texts.

---

## BLE

Advertising after GATT disconnects is reasserted deterministically so ESPHome's automatic advertiser restart cannot replace the Sensirion manufacturer-data payload. The Device Settings privacy flag (`IsAdvertiseDataEnabled`) is persistent and is logged explicitly at startup.
 history

With:

```yaml
co2_monitor_0601:
  ble_history: true
```

samples are stored in the `senshist` flash partition.

MyAmbience can download the history through the compatible GATT protocol.

The history ring is persistent across normal reboots.

The 4096-sample history is flash-backed. Only a small pending write ring is kept in RAM, so enabling MyAmbience history does not require a second 32 KiB in-memory copy of the persistent sample store.

---

# Experimental secure MyAmbience settings

This part of the project is under active reverse engineering and is not required for normal sensor operation.

The firmware exposes a Sensirion-compatible Device Settings service:

```text
Service   0x8100
```

with the known SHT43 DemoBoard characteristics:

```text
0x81FF  Settings Version                   read
0x81FE  IsLogEnabled                       read/write
0x8130  IsAdvertiseDataEnabled / Privacy   read/write
0x8120  AlternativeDeviceName              read/write, max 31 bytes
```

The service is created directly with the ESP-IDF GATT server API rather than the ESPHome characteristic wrapper. This is intentional: all four characteristics require encrypted MITM-authenticated access, matching the security flags used by Sensirion's SHT43 DemoBoard firmware. The BLE stack is configured for LE Secure Connections, bonding, Numeric Comparison and 10–16 byte encryption keys.

Settings are persisted in ESPHome NVS. `IsAdvertiseDataEnabled` is applied at runtime. `AlternativeDeviceName` is retained while the GAP identity remains unchanged so the compatibility classification is not accidentally altered. `IsLogEnabled` is retained as an experimental direct-BLE alias for the WiFi/Home Assistant disable state; Sensirion does not define that meaning, and MyAmbience does not expose this SHT43-only switch for a normal MyCO2/DIY identity.

Because the ESP installation has neither a dedicated display nor a physical confirmation button, pairing authorization is provided through Home Assistant. Enable:

```text
BLE Pairing Mode
```

immediately before pairing in MyAmbience. The authorization window defaults to 60 seconds. If MyAmbience is already connected when the switch is enabled, the firmware immediately requests authenticated encryption for that peer; connections established during the window do the same on connect. Numeric Comparison is accepted only while the authorization window is open. After successful authentication, or when the timeout expires, Pairing Mode is switched off.

This is not equivalent to the original SHT43 DemoBoard's physical numeric-comparison confirmation and should be regarded as an experimental ownership gate. Existing bonded peers may resume encrypted sessions without reopening the pairing window.

---

# SHT43 identity probe

`i2c-sniffer-sht43-probe.yaml` deliberately makes the ESP appear to MyAmbience as an SHT43 DemoBoard.

It is used only to study how MyAmbience selects device-specific functionality.

The probe currently uses:

```text
GAP name:       SHT43 DB
Sample type:    0x06
Test device ID: 68:43
```

and exposes SHT43-style temperature, humidity and Device Settings GATT services.

This experiment confirmed that MyAmbience switches to its SHT43-specific UI when the SHT43 identity is advertised. The app then exposes:

- Device Information
- Device Name
- Logging Interval
- Sensor Certificate
- Privacy

MyAmbience also treats history in this mode as temperature/humidity history, so CO₂ values stored in the same internal history are not displayed.

The production firmware should therefore continue to use the normal SCD-compatible identity.

Future experiments may investigate whether separate logical SHT43 and SCD identities can coexist so that one exposes secure settings while the other remains visible as a CO₂ gadget.

---

# Passive CO₂ decoding

The CO₂ channel is passively sniffed from the original I²C bus.

The observed module uses address:

```text
0x62
```

and the relevant transaction corresponds to the SCD4x-style measurement command:

```text
0xEC05
```

The decoder validates:

- I²C addressing
- read/write direction
- expected command bytes
- ACK/NACK structure
- complete response length
- Sensirion CRC

Malformed or incomplete captures are rejected rather than converted into measurements.

---

## Missed-edge recovery

GPIO-based passive sniffing on the ESP32-C3 occasionally loses a complete SCL pulse.

Real captures showed that this can produce an exact one-bit shift in an otherwise valid transaction.

The decoder therefore contains a deliberately conservative recovery mechanism.

It can attempt to reconstruct at most two missing SCL clocks per capture.

Candidate locations are generated only from unusually long periods in which captured SCL remains at a constant level.

Timing is used only to generate possible hypotheses.

A recovery is accepted only if all valid hypotheses produce the same complete CO₂ transaction with:

- address `0x62`
- command `0xEC05`
- correct ACK/NACK behavior
- valid Sensirion CRC

If valid hypotheses disagree, recovery is rejected.

The original GPIO waveform is retained unchanged for diagnostics.

---

# Raw debug capture

With:

```yaml
co2_monitor_0601:
  debug_capture: true
```

the debug web server exposes:

```text
/capture
/rt_rh_capture.csv
/rt_rh_timing.csv
```

The first suspicious I²C transaction is frozen until successfully downloaded.

UDP debug export uses 512-byte payload chunks. If lwIP reports `ENOMEM`, the
export remains pending and retries after a short backoff instead of repeatedly
calling `sendto()` on every component loop. The warning includes free/largest
8-bit heap, minimum free heap, internal heap, and the cumulative ENOMEM count.
This instrumentation is intended to distinguish general heap exhaustion from
lwIP/pbuf pressure while preserving the raw capture for a later retry.

Interesting conditions include:

- malformed framing
- unhandled transactions
- protocol-invalid CO₂ frames
- software-recovered missing clocks

A successfully resolved coalesced SDA/SCL sample alone does not consume the freeze slot.

The current raw format is:

```text
LA02
```

which includes the bus state before the first captured edge.

The supplied converter:

```text
tools/capture2vcd.py
```

supports `LA02` and the older `LA01` formats.

Captured transactions are normally well below 1 KiB and are sent synchronously by the ESP-IDF HTTP server.

---

# RT/RH decoding

Temperature and humidity are derived from timing relationships observed on the original Unni RT/RH circuitry.

The decoder measures:

- reference timing
- RT timing
- RH state timing
- signal counts and duration
- derived timing ratios

Calibration is specific to the tested device family.

Diagnostic builds expose additional information including:

- measurement quality
- timing ratios
- logarithmic RH ratio
- temperature rate of change
- extrapolation flags
- thermal-transient detection

The active calibration is defined in:

```text
co2_monitor_0601/calibration.h
```

---

# Startup behavior

The ESP deliberately leaves the sniffing GPIOs untouched during the initial startup delay:

```yaml
co2_monitor_0601:
  sniffer_start_delay: 10s
```

This reduces the risk that ESP GPIO initialization influences the original Unni electronics during their own startup sequence.

After the delay, passive capture is enabled.

The shared GPIO ISR service is installed IRAM-safe before Wi-Fi initialization, while the actual sensor GPIO capture remains isolated until the startup delay has elapsed.

---

# Repository layout

```text
co2_monitor_0601/
  __init__.py
      ESPHome schema and code generation

  co2_monitor_0601.cpp/.h
      component orchestration
      Home Assistant publishing
      USB/battery policy

  i2c_sniffer.cpp/.h
      passive GPIO I²C capture and framing

  co2_decoder.cpp/.h
      CO₂ protocol and CRC validation

  rtrh_decoder.cpp/.h
      RT/RH timing decoder

  power_save.cpp/.h
      CPU frequency and Light-Sleep policy

  calibration.h
      RT/RH calibration

  sensirion_ble.cpp/.h
      BLE live advertising

  sensirion_history.cpp/.h
      persistent BLE history and GATT download

  sensirion_settings.cpp/.h
      experimental Device Settings support

  sensirion_sht43_probe.cpp/.h
      experimental SHT43 identity probe

docs/
  DEVELOPMENT_HISTORY.md
      development process and design rationale

  ISR_ARCHITECTURE.md
      timing-critical GPIO/ISR architecture

  LIGHT_SLEEP.md
      power-management implementation

  history/
      detailed notes about superseded approaches

tools/
  reverse-engineering and capture utilities
```

Historical documents may describe earlier pin assignments, names or implementations. The current source, README and active documentation are authoritative.

---

# Safety

Verify polarity, voltage levels and actual PCB connections before attaching the XIAO.

The ESP and Unni electronics share ground, and mistakes on battery, VBUS or signal connections can damage either device.

This project is independent community work and is not affiliated with the manufacturer of the Unni monitor or with Sensirion.

---

# License

Project-authored code and documentation are licensed under:

```text
GPL-3.0-or-later
```

See [LICENSE](LICENSE).

Referenced or adapted Sensirion-compatible portions retain the required upstream notices and provenance information.

See:

- [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
- `LICENSES/`


### SHT43 sniffer-off A/B diagnostic

The `i2c-sniffer-sht43-probe.yaml` variant in this diagnostic build sets `sniffer_enabled: false`. This keeps Wi-Fi, Home Assistant, BLE advertising, Sensirion history, and the SHT43 serial/T/RH GATT services active while completely skipping installation of the shared GPIO ISR service and all CO2/RT/RH capture/decoder GPIO setup. It is intended only to isolate BLE/GATT behavior from the passive sniffer ISR path.


### RT/RH GPIO-without-ISR diagnostic

The `rtrh_gpio_setup` option is a diagnostic A/B switch. When enabled with `rtrh_enabled: false`, the RT/RH pins and power-save wake configuration are initialized while the RT/RH edge ISR and decoder remain disabled. It is intended only for isolating ESP32-C3 radio/capture scheduling conflicts.


## Diagnostic build: RT/RH publication A/B

The `i2c-sniffer-sht43-probe.yaml` variant enables the normal RT/RH decode and publication path again. The earlier Device Settings omission was an A/B diagnostic during the RAM investigation; after converting the 4096-sample history to flash-backed storage, the secure 0x8100 service and BLE Pairing Mode switch are enabled again.


## Diagnostic build: known-good RT/RH restore

This A/B build restores `rtrh_decoder.cpp` and `rtrh_decoder.h` byte-for-byte from the known-good rebase that produced valid RT/RH measurements on 2026-08-15 around 16:05. The current full SHT43 GATT, Wi-Fi scan policy and RAM-headroom test configuration are retained. The purpose is to test the RT/RH GPIO/ISR initialization regression before changing the humidity decoding algorithm.

### BLE privacy and Wi-Fi / Home Assistant switches

Two runtime controls are exposed to Home Assistant when HA entities are enabled:

- **WiFi / Home Assistant** disables the ESPHome Wi-Fi interface after a short grace period. The state is restored by ESPHome's switch restore mode. Because HA cannot reach a device whose Wi-Fi is off, connecting USB power while the switch is OFF opens a 5-minute recovery window; turn the switch ON during that window to keep Wi-Fi enabled.

The example Wi-Fi/API configurations set `reboot_timeout: 0s`, otherwise ESPHome's normal connectivity watchdog could reboot a device that intentionally has Wi-Fi disabled.


### MyAmbience Device Settings compatibility

Reviewing the bundled Sensirion repositories shows that MyAmbience settings are device-type specific rather than purely characteristic-driven.

For the **SHT43 Demo Board**, Sensirion documents `0x81FE IsLogEnabled`, `0x8130 IsAdvertiseDataEnabled` (Privacy), `0x8120 AlternativeDeviceName`, and `0x81FF DeviceSettingsVersion`. For a **DIY Gadget**, the current Sensirion service specification and Arduino server instead list `0x8120 AlternativeDeviceName`, `0x8171 Wi-Fi SSID`, and `0x8172 Wi-Fi Password`.

This matches the earlier identity probe: MyAmbience exposes Privacy when the device advertises as an SHT43 Demo Board, but that identity makes the app treat the history as T/RH-only and loses the desired MyCO2 presentation. Production therefore keeps the MyCO2-compatible identity.

`0x8130` remains available for protocol experiments and direct GATT clients. Its privacy advertisement follows Sensirion's SHT43 implementation: the short Sensirion manufacturer header remains present with advertisement type `0xFF` and sample type `0x00`, while live measurement bytes are omitted.

`0x81FE` remains an **experimental direct-BLE alias** for disabling WiFi/Home Assistant. MyAmbience is not expected to render this SHT43-only setting as a switch for the production MyCO2 identity. The Home Assistant `WiFi / Home Assistant` switch and USB recovery window remain the supported UI for that function.

The official DIY Wi-Fi credential characteristics (`0x8171`/`0x8172`) are not exposed yet: accepting credentials without safely integrating them into ESPHome's configured-network lifecycle would create a setting that appears functional but is not reliable.
