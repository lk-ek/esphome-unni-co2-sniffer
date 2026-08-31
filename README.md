<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Unni CO₂ Smartification and Sensirion Gadget Bridge

ESPHome components and example firmware for two related sensor integrations:

- a Seeed Studio XIAO ESP32-C3 added to an Unni CO₂ monitor without taking control away from the original electronics; and
- a stationary, externally powered AHT21 room sensor, optionally combined with an ENS160, which reuses the same Sensirion/MyAmbience BLE and persistent-history implementation.

In the Unni build, the XIAO ESP32-C3 acts primarily as a **passive sniffer**:

- CO₂ is decoded from the existing I²C traffic between the Unni controller and the CO₂ module.
- Temperature and relative humidity are reconstructed from the existing RT/RH timing signals.
- Measurements are exposed to Home Assistant through ESPHome.
- The same measurements can be advertised over BLE in a Sensirion MyAmbience-compatible format.
- Optional persistent BLE history allows historical samples to be downloaded with MyAmbience.

The tested Unni monitor is sold as **CO2 Monitor Carbon Dioxide Detector 0601**, hence the orchestrator component name. BLE and history are provided by the separate reusable bridge:

```yaml
sensirion_gadget_bridge:
  id: unni_sensirion_bridge
  profile: trh_co2
  device_name: "Unni CO2 Monitor"

co2_monitor_0601:
  sensirion_bridge_id: unni_sensirion_bridge
```

The Unni integration was developed against one specific hardware revision. Other revisions should be verified before connecting the ESP.

---

## Supported devices

### Unni monitor

The passive in-monitor integration provides:

- passively decoded CO₂ concentration;
- reconstructed temperature and relative humidity;
- battery voltage and estimated battery state of charge;
- USB/VBUS presence;
- automatic USB/battery power policy and Light Sleep; and
- optional diagnostics and raw capture.

### Standalone room sensor

The stationary, externally powered integration provides:

- AHT21 temperature and relative humidity;
- optional ENS160 TVOC and AQI as Home Assistant-only air-quality measurements;
- SHT43-compatible BLE live data and persistent history; and
- no battery, VBUS or Unni capture subsystem.

ENS160 eCO₂ is deliberately not published. It is an algorithmic estimate, not the direct CO₂ measurement required by the `trh_co2` profile.

### Shared integrations

- ESPHome / Home Assistant
- Sensirion-compatible BLE live advertisements
- persistent BLE history with MyAmbience-compatible GATT download

### Unni-only power management

The following features belong to the Unni/XIAO integration:

- automatic distinction between USB and battery operation
- ESP32-C3 dynamic frequency scaling
- automatic Light Sleep in battery mode
- reduced BLE advertising rate on battery
- reduced Home Assistant publication rate on battery
- optional `Energy Save Mode` for measuring battery-style firmware behavior while physically powered through USB
- dedicated BLE-only measurement build without Wi-Fi or Home Assistant

### Unni diagnostics

Optional debug builds provide:

- RT/RH timing diagnostics
- I²C framing and decoder diagnostics
- raw I²C captures
- VCD conversion tools
- recovery diagnostics for occasionally missed GPIO edges

---

# Unni monitor integration

Everything in this section concerns the passive XIAO ESP32-C3 installation in
the Unni monitor or its future ESP32-C6 replacement PCB. The standalone room
sensor is documented separately below.

## ESP32-C6 Rev A PCB — work in progress

The [`unni-smartification-c6/`](unni-smartification-c6/) subtree contains a
dedicated four-layer ESP32-C6 hardware design with a replacement USB/power board
and a separate ESP/battery board. It keeps the original Unni connections as
T-taps and adds autonomous charging, USB/battery source isolation, native USB
and a hardware-inhibited forced-awake 5 V path.

> [!WARNING]
> This PCB is a **work in progress**. Placement and routing are not final, and
> the design has not replaced the currently tested XIAO ESP32-C3 installation.
> Reproducible schematic, connectivity, mechanical and system-level SPICE checks
> reduce design risk but do not constitute assembled-hardware validation.

The prototype PCBs for this project are kindly sponsored by
[PCBWay](https://www.pcbway.com/). Thank you to PCBWay for supporting the
project and helping turn the design into physical prototypes.

<a href="https://www.pcbway.com/"><img src="unni-smartification-c6/docs/images/pcbway-logo.png" alt="PCBWay" width="320"></a>

See the [ESP32-C6 Rev A project README](unni-smartification-c6/README.md) for
the power architecture, mechanics, current routing/fabrication status and
validation commands.

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

The current tested installation uses **10 kΩ series resistors on each passive ESP tap**:

```text
Unni signal ---- 10 kΩ ---- XIAO GPIO
```

The resistors belong only in the ESP branches; do not insert them into the original Unni signal path. The RT/RH calibration is hardware-dependent, so calibration data collected with earlier direct or three/four-wire tap arrangements must not be mixed with the current two-wire/10 kΩ setup.

The RT/RH decoder now keeps separate temperature views: a diagnostic RT-model temperature, a dedicated Unni LCD-emulation curve, and a provisional external-reference Air Temperature curve. RH uses the RH-phase carrier period normalized by REF with temperature compensation. Air Temperature is the canonical temperature exported through Sensirion BLE/history when it is inside its validated envelope; the RT model is retained as a fallback outside that envelope. See `co2_monitor_0601/calibration.h` and the dated calibration notes under `docs/history/` for the coefficients and fit provenance.

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

The runtime and charge-time estimates are learned from the observed voltage trend rather than from a fixed assumed current draw. Each USB/battery transition starts a new rate-estimator session, followed by a two-minute settling period. The short-term estimator then uses at least a five-minute observation window and exponentially smooths subsequent rate measurements. Until a meaningful trend is available, the corresponding estimate remains unavailable instead of publishing a speculative value.

Battery runtime also has a persistent self-learning layer. During battery operation the component accumulates real elapsed time and voltage-derived SOC drop. Once a session contains at least 2 hours and 8 percentage points of discharge, it can estimate an equivalent full-runtime value. A completed qualifying battery session is folded into the persistent model with a conservative EMA (25% new session / 75% previous model). The active session and learned model are checkpointed to flash every `battery_learning_save_interval` (default `30min`) and on session completion, so an overnight run or reboot does not discard the calibration data. The production `Battery Runtime Estimate` blends the persistent model with the recent discharge-rate estimate; before a learned model exists it behaves like the original recent-rate estimator.

With `debug_metrics: true`, `Battery Learned Full Runtime`, `Battery Learning Progress`, and `Battery Learning Cycles` expose the learning state. `Battery Learning Progress` reaches 100% when the current session has both at least 2 hours of data and at least 8 percentage points of SOC drop. The model intentionally learns effective runtime rather than claiming to measure true battery capacity in mAh; without a coulomb counter the voltage/SOC curve cannot provide that measurement reliably.

Battery-learning flash writes are logged after a successful synchronous commit.
Periodic checkpoints are labeled `periodic`, OTA-triggered force-saves are
labeled `OTA`, and end-of-session commits are labeled `session-end`, together
with the persisted SOC/session runtime/progress and learned-runtime state.

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

# Unni installation

## Minimal configuration

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

sensirion_gadget_bridge:
  id: unni_sensirion_bridge
  profile: trh_co2

co2_monitor_0601:
  sensirion_bridge_id: unni_sensirion_bridge
```

The repository contains complete example configurations, so in normal use it is easier to start with one of those rather than build the YAML from scratch.

Build or flash with ESPHome:

```bash
esphome compile i2c-sniffer.yaml
esphome run i2c-sniffer.yaml
```

# Unni build variants

The repository contains these configurations for the Unni monitor:

| Configuration | Wi-Fi / HA | BLE | Debug capture | Purpose |
|---|---|---|---|---|
| `i2c-sniffer.yaml` | yes | yes | no | normal operation |
| `i2c-sniffer-debug.yaml` | yes | yes | yes | protocol/debugging work |
| `i2c-sniffer-no-ble.yaml` | yes | no | no | BLE power comparison |
| `i2c-sniffer-ble-only.yaml` | no | yes | no | BLE-only power measurement |
| `i2c-sniffer-sht43-probe.yaml` | yes | experimental | no | MyAmbience reverse engineering |

The SHT43 probe is diagnostic firmware and should not be used as the normal
Unni device identity.

---

# Unni Home Assistant entities

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

The normal `i2c-sniffer.yaml` build intentionally exposes only the production
measurement set plus operational power/battery controls:

The default Home Assistant names are intentionally concise: the physical-air `air_temperature` entity is shown as `Temperature`, and the physical carrier-based `rh_humidity` entity as `Humidity`.

- `CO2`
- `Temperature` (physical-air estimate)
- `Humidity` (physical carrier-based estimate)
- `Battery Voltage`
- `Battery Level`
- `Battery Runtime Estimate`
- `Battery Charge Time Estimate`
- `USB Power`
- `Energy Save Mode`
- `WiFi Home Assistant` when the runtime connectivity control is enabled
- `BLE Privacy` and `BLE Pairing Mode` while the experimental secure MyAmbience settings support is enabled

Sensor definitions may be customized:

```yaml
co2_monitor_0601:
  co2:
    name: "Living Room CO2"

  air_temperature:
    name: "Temperature"

  rh_humidity:
    name: "Humidity"
```

The shipped `i2c-sniffer-debug.yaml` sets `debug_metrics: true` and creates the
complete entity set as well: `RT Temperature`, both `Unni Display ...`
emulation entities, protocol error counters, raw timing/ratio metrics, battery
charge/discharge rates, quality and calibration/extrapolation flags. These are
not instantiated by the normal build.

# Unni component configuration

Useful options include:

```yaml
co2_monitor_0601:
  # Home Assistant
  home_assistant: true
  ha_publish_interval: 60s

  # Bridge composition and battery-specific BLE interval
  sensirion_bridge_id: unni_sensirion_bridge
  ble_battery_advertising_interval: 3s

  # Power management
  light_sleep: true
  light_sleep_max_awake: 10s
  co2_wake_idle_stable: 500ms
  co2_wake_guard_time: 500ms
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
  battery_learning_save_interval: 30min
  usb_power_pin: 5

  # Startup
  sniffer_start_delay: 10s

  # Diagnostics
  debug_metrics: false
  # Compiles main-loop stage timing and heap telemetry into the firmware.
  runtime_diagnostics: false
  # Optional ESP32-C3/C6 hardware assist. "gpio" keeps the pure ISR backend;
  # "rmt_scl" records SCL in RMT hardware and uses it only to repair strictly
  # protocol-invalid GPIO captures.
  i2c_capture_backend: gpio
  debug_capture: false
  # Experimental only: temporarily becomes an I2C master on a long-idle
  # LOW/LOW CO2 bus, using only the ESP32-C3 internal pull-ups through the
  # external 10 kOhm tap resistors.
  active_i2c_probe: false
  active_i2c_probe_interval: 60s
```

The defaults match the tested XIAO ESP32-C3 installation.

All configured GPIOs must be unique.

`rtrh_enabled`, `rtrh_edge_capture`, and `rtrh_decode_only` are mutually
exclusive capture modes. `rtrh_gpio_setup: true` is accepted only with one of
those modes; invalid combinations fail during YAML validation rather than
retrying GPIO setup at runtime.

`battery_pin` must be an ESP32-C3 ADC1-capable GPIO.

The Unni orchestrator configures its ESP-IDF power-management requirements;
the bridge configures the BLE server and persistent-history partition.

`active_i2c_probe` is intentionally disabled by default. It is a hardware diagnostic, not normal passive-sniffer operation. The probe runs only on battery policy after the CO2 bus has remained LOW/LOW and edge-free for at least one second. It first tries `0xEC05`; if no slave ACKs, it tries the observed `0x21B1` start command. Every attempt restores SDA/SCL to input-only/no-pull mode immediately afterwards. Do not enable it without series resistance on both CO2 tap lines.

---

# Unni USB and battery power policy

The firmware uses different runtime policies depending on physical power.

## USB power

Normal USB operation prioritizes responsiveness and reliability:

- automatic MCU Light Sleep is prevented
- CPU frequency is held at 80 MHz
- Wi-Fi power saving is disabled at runtime with `WIFI_PS_NONE`
- CO₂ I²C capture remains continuously enabled
- valid CO₂ measurements are published immediately
- valid RT temperature is published immediately; RH is published when its carrier timing also validates
- BLE advertisements use the USB interval, default `2s`

Because USB power is available, reducing ESP consumption is not the priority in this mode.

## Battery power

Battery operation prioritizes low average power:

- automatic Light Sleep is enabled
- GPIO3/GPIO4 RT/RH activity wakes the ESP
- CPU frequency is temporarily raised to 80 MHz while measurements are captured
- during a powered CO₂ measurement window the passive sniffer is active; after two plausible CO₂ observations it is gated off before the sensor rail collapses
- the expected slow SCL/SDA shutdown decay is ignored with the I²C interrupts disabled, avoiding large garbage captures and ISR storms
- after the dead bus has remained LOW/LOW for `co2_wake_idle_stable` (default `500ms`) plus `co2_wake_guard_time` (default `500ms`), GPIO7/SCL HIGH is armed as the next CO₂ power-up wake source
- a real CO₂ power-up disarms GPIO7 wake and re-enables the passive sniffer before the native measurement transaction; GPIO6 remains passive
- after the RT/RH wake window is complete, the ESP may return to Light Sleep
- Wi-Fi uses `WIFI_PS_MIN_MODEM`
- Home Assistant publication is throttled to the latest values, default once per minute
- BLE advertisements use the battery interval, default `3s`

The corresponding options are:

```yaml
sensirion_gadget_bridge:
  id: unni_sensirion_bridge
  advertising_interval: 2s

co2_monitor_0601:
  sensirion_bridge_id: unni_sensirion_bridge
  light_sleep: true
  light_sleep_max_awake: 10s
  co2_wake_idle_stable: 500ms
  co2_wake_guard_time: 500ms
  ha_publish_interval: 60s
  ble_battery_advertising_interval: 3s
```

---

# Unni Energy Save Mode

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

# Unni BLE-only measurement build

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
sensirion_gadget_bridge:
  id: unni_sensirion_bridge
  profile: trh_co2
  ble: true
  ble_live: true
  ble_history: true

co2_monitor_0601:
  sensirion_bridge_id: unni_sensirion_bridge
  home_assistant: false
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

# Standalone room sensor integration

This is a stationary, externally powered AHT21 room sensor. An ENS160 may be
fitted for additional Home Assistant air-quality data. The existing
`mobilesensor-*.yaml` filenames are retained for compatibility, but “mobile
sensor” is not used here as the device description because the hardware has no
battery operating mode.

It uses `sensirion_gadget_bridge` directly and does not instantiate
`co2_monitor_0601`. Consequently it has no Unni GPIO capture, battery ADC,
battery learning, VBUS detection, Energy Save Mode or automatic Light-Sleep
policy.

## Hardware

The shipped configurations target `wemos_d1_mini32` and use one 100 kHz I²C
bus:

| GPIO | Device | Address | Purpose |
|---:|---|---:|---|
| GPIO23 | AHT21 and optional ENS160 | `0x38` / `0x53` | SDA |
| GPIO19 | AHT21 and optional ENS160 | `0x38` / `0x53` | SCL |

No battery divider, VBUS input or Unni signal tap is part of this configuration.

## Configuration

```yaml
sensirion_gadget_bridge:
  id: sensirion_bridge
  profile: sht43_trh
  temperature_id: aht21_temp
  humidity_id: aht21_humi
  history_time_id: room_sensor_wall_clock
  ble: true
  ble_live: true
  ble_history: true
  advertising_interval: 2s
```

Use `mobilesensor-sensirion.yaml` when an ENS160 is fitted, or
`mobilesensor-sensirion-no-ens160.yaml` for AHT21-only hardware. Both variants
provide the same AHT21 BLE identity, live data, persistent history, OTA flush
and MyAmbience GATT behavior.

The bridge coalesces sequential AHT21 temperature and humidity callbacks for
100 ms. A restarted coalescing window publishes the newest complete pair rather
than two partially updated samples.

## Home Assistant and ENS160

Both variants expose AHT21 temperature and humidity. The ENS160 variant also
exposes TVOC, AQI and an `ENS160` control switch. It starts off after every
boot. Enabling it wakes the sensor before polling resumes; disabling it suspends
polling before Deep Sleep and publishes TVOC/AQI as unavailable.

AHT21 compensation remains enabled for ENS160 operation. Heating from an active
ENS160 may influence the nearby AHT21; no unvalidated correction curve is
applied. TVOC and AQI never participate in BLE sample completeness, GATT or
history. ENS160 eCO₂ is exposed nowhere. Switching the ENS160 on or off cannot
pause, clear or reset AHT21 history.

---

# Shared Sensirion bridge

## Architecture and sample ownership

BLE and history consume one authoritative, profile-aware sample owned by
`sensirion_gadget_bridge`:

```yaml
sensirion_gadget_bridge:
  id: unni_sensirion_bridge
  profile: trh_co2       # or sht43_trh
  ble: true
  ble_live: true
  ble_history: true
  device_name: "Unni CO2 Monitor"
  identity_mode: device_derived
  advertising_interval: 2s
  history_time_id: unni_wall_clock
```

For `sht43_trh`, `temperature_id` and `humidity_id` may bind the bridge directly
to ESPHome sensors. They must be configured together.
`sht43_identity_probe: true` remains a compatibility alias for this profile;
contradictory profile and alias settings are rejected.

| Concern | Unni monitor | Standalone room sensor |
|---|---|---|
| Measurement producer | passive CO₂ and RT/RH decoders | AHT21 state callbacks |
| Bridge profile | `trh_co2` | `sht43_trh` |
| Complete sample | finite T/RH plus valid CO₂ | finite T/RH; CO₂ remains unset |
| History guard | Unni capture-aware guard | generic transfer watchdogs only |
| Optional air quality | none | ENS160 TVOC/AQI in HA only |
| Power policy | USB/battery orchestration | externally powered; no Unni policy |

Invalid or out-of-range input does not replace the last valid sample. BLE live
data, SHT43 GATT temperature/humidity and history all read the same accepted
sample, preventing different paths from applying different completeness rules.
The first complete sample is immediately eligible for history; subsequent
samples follow the configured history interval. Existing flash layout and GATT
history wire format are shared by both producers.

## Sensirion / MyAmbience compatibility

The normal Unni firmware advertises temperature, humidity and CO₂ in a Sensirion Gadget/MyAmbience-compatible format. The standalone room-sensor profile advertises SHT43-style temperature and humidity only.
For the temperature field it uses `Air Temperature` inside the externally validated air-temperature envelope, falling back to the diagnostic RT model only when that physical-air estimate is unavailable. BLE humidity remains the physical carrier-based `RH Humidity`; the Unni display-emulation values are never advertised as physical measurements.

The reusable bridge is composed with the Unni orchestrator in the shipped BLE
YAMLs. The name shown for the normal MyCO2 gadget is configured on that bridge:

```yaml
sensirion_gadget_bridge:
  id: unni_sensirion_bridge
  profile: trh_co2
  device_name: "Unni CO2 Monitor"

co2_monitor_0601:
  sensirion_bridge_id: unni_sensirion_bridge
```

`device_name` defaults to `Unni CO2 Monitor` for the `trh_co2` profile and to `SHT43 DB` for the `sht43_trh` profile. It is limited to 31 bytes to match the Sensirion `AlternativeDeviceName` characteristic. It sets the normal Device Information model name and the default `AlternativeDeviceName`. A name explicitly changed through the Device Settings service remains persistent. The compatibility-sensitive GAP/local identity remains `S` and is intentionally not changed by this option. The old `ble_device_name` key remains on the Unni compatibility surface.

Bridge `identity_mode` defaults to `device_derived` and therefore needs no YAML
configuration. It derives a stable, locally administered Bluetooth address and
Device ID from the ESP eFuse MAC so multiple monitors do not collide.
`legacy_fixed` remains available only as an explicit compatibility override for
installations that must retain the old fixed identity. Changing identity can
invalidate existing bonds and MyAmbience caches and may require opening Pairing
Mode once in Home Assistant.

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

## Persistent BLE history

Advertising after GATT disconnects is reasserted deterministically so ESPHome's automatic advertiser restart cannot replace the Sensirion manufacturer-data payload. The Device Settings privacy flag (`IsAdvertiseDataEnabled`) is persistent and is logged explicitly at startup.

With:

```yaml
sensirion_gadget_bridge:
  ble_history: true
```

samples are stored in the `senshist` flash partition.

MyAmbience can download the history through the compatible GATT protocol.

The history ring is persistent across normal reboots.

The 4096-sample history is flash-backed. Only a small pending write ring is kept in RAM, so enabling MyAmbience history does not require a second 32 KiB in-memory copy of the persistent sample store. When a continuous run exceeds 4096 samples, the sparse time anchor advances with each ring eviction; restored history therefore still timestamps the retained 4096-sample window rather than the already-overwritten beginning of the run.

History metadata uses a redundant version-4 A/B journal in the first and last
sectors of the 64 KiB partition; the 14 data sectors and 8-byte sample wire
layout are unchanged. Version-2/3 metadata is read in place and migrated on the
next metadata flush. V4 uses the two formerly reserved metadata words for a
sparse wall-clock anchor and the sample count of the newest continuous run. The
anchor is created only when a run starts after a reboot/cadence gap, when the
wall clock first becomes valid, or after a material wall-clock correction; it is
not stored per sample. API-enabled variants obtain UTC from Home Assistant. The
BLE-only build has no wall-clock source and therefore starts a new relative run
after reboot rather than inventing timestamps. Failed metadata writes remain
dirty and are retried. A client may select only intervals from 60 seconds
through 24 hours. Changing the interval clears history incrementally from the
main loop (at most one sector erase per loop), while sampling and download
remain paused.

The official Sensirion download format has one header with one interval and one
`age-of-latest-sample`, so it cannot describe gaps inside a transfer. MyAmbience
therefore receives only the newest continuous run (or the newest requested
subset of that run). Older samples remain stored in the flash ring but are not
misrepresented as contiguous data after a gap.

On Unni, an injected producer guard cooperatively pauses history packet production around the
approximately six-second CO2 and 30-second RT/RH rhythms. Completed measurements
continuously adapt both predicted periods; sending stops 800 ms before either
prediction and for at least 150 ms afterwards, allowing queued BLE work to drain.
Any observed I2C or RT/RH capture blocks sending reactively, with a 750 ms
failsafe, so a malformed/stuck capture cannot starve BLE forever. The connection,
CCCD subscription, and sample cursor remain intact during ordinary pauses. A
transfer is aborted without deleting history after 120 seconds total or 15
seconds without queueing a notification.

The 800 ms lead is adaptive only while a download is active. A raw CO2 capture
with fewer than 130 SCL transitions or any frame error adds 250 ms, capped at
500 ms extra. Every three consecutive clean captures remove 50 ms until the
800 ms baseline is restored. Completion and abort logs include guard pause count/time,
CO2 captures observed during the transfer, damaged-capture count, minimum raw
edge count, and the final adaptive pre-guard. This makes a real 4096-sample
MyAmbience field download directly auditable without enabling high-rate debug
transport. RMT-SCL assist remains the permanent recovery
layer for occasional missed GPIO edges both inside and outside history traffic.
The standalone room-sensor bridge supplies no Unni guard; its history transport still retains
the generic connection/cursor behavior and the 120-second/15-second watchdogs.

---

## Experimental secure MyAmbience settings

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

# Appendix: Unni technical reference

The following decoder, capture and wake details apply only to the Unni monitor.
They do not apply to the standalone AHT21 room sensor.

## SHT43 identity probe

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

## Passive CO₂ decoding

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

### Missed-edge recovery

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

## Raw debug capture

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

## RT/RH decoding

Temperature and humidity are reconstructed from timing signals already present in the original Unni RT/RH circuitry. The ESP is a passive observer; it does not drive either line.

The current hardware calibration applies to the **two-wire RT/RH tap with 10 kΩ series resistance on both ESP branches**. Earlier captures made with direct taps or additional connected signal lines remain useful for reverse engineering, but they are not calibration-compatible with the current installation.

### Temperature

The decoder identifies fixed REF and RT phases by elapsed time and derives temperature from the ratio of the RT period to the reference period. REF/RT validation is independent from the RH tail, so a trustworthy temperature can still be published when humidity decoding fails.

The active calibration is defined in:

```text
co2_monitor_0601/calibration.h
```

Temperature is maintained as three deliberately separate views so different calibration goals are not mixed:

- **`Air Temperature`** is the production physical-air estimate, fitted only to nearby/same-airflow AHT21/BME280 reference points. Its current supported ratio envelope is `1.98..2.35`. Sensirion BLE live data and persistent history use this physical-air value inside the supported envelope and fall back internally to the RT model only outside it. This is the only temperature entity created by the normal build.
- **`RT Temperature`** keeps the existing v2 RT/REF conversion as an internal/diagnostic raw-model temperature and remains the temperature input to the already-fitted carrier-RH compensation. It is exposed only with `debug_metrics: true`.
- **`Unni Display Temperature`** is an independent quadratic LCD-emulation fit and is exposed only with `debug_metrics: true`.
- **`RH Humidity`** remains the production physical carrier-based humidity estimate.
- **`Unni Display Humidity`** is a separate provisional LCD-emulation fit using carrier/REF plus the independently reconstructed Unni display temperature. It does not feed back into the physical RH path and is exposed only with `debug_metrics: true`.

The display and air curves are therefore intentionally not interchangeable. The LCD emulation answers “what would the Unni display show?”, while the air estimate answers “what air temperature is most consistent with the nearby external references?”.

### Humidity

Humidity is decoded from the **RH-phase carrier period normalized by REF**:

```text
rh_ratio = rh_carrier_period / ref_period
x = ln(rh_ratio)
RH = A*x^2 + B*x + C*T + D
```

This replaces the older recurrence-based `RT=0, RH=1` state decoder. Around 60–70 %RH the RT and RH carriers become nearly phase-aligned, making that short combined state disappear from GPIO sampling even though both physical carrier signals remain clean. The carrier period itself remains directly measurable across the tested range, including those high-humidity captures.

The initial carrier-v1 calibration is based on measurements spanning roughly 30–68 %RH and intentionally heated temperature sweeps. It is suitable for production testing but should still be refined with additional stationary calibration points. The old RH-state interval, its distribution, edge counts, and direct RT↔RH phase offsets remain available as diagnostics and no longer determine whether humidity is valid.

Debug builds also report:

- RH carrier period and `carrier/REF` ratio;
- RT and RH rising/falling edge counts during the RH phase;
- the legacy combined-state recurrence timing for comparison;
- direct signed RT↔RH edge separation for corresponding rising and falling edges.

For the direct phase diagnostic, positive `RH-RT` values mean the RH edge followed the corresponding RT edge; negative values mean RH led RT. Corresponding edges are paired only when they occur within 35 µs. Phase is diagnostic only; the production humidity observable is the carrier period.

With `debug_metrics: true`, Home Assistant exposes the complete diagnostic entity set: RT-model and Unni-display values, protocol error counters, timing/ratio metrics, battery-rate metrics, quality, temperature-rate and calibration/extrapolation flags. With `debug_capture: true`, raw and timing data are available over UDP and the debug HTTP endpoints.

---

## Startup behavior

The ESP deliberately leaves the sniffing GPIOs untouched during the initial startup delay:

```yaml
co2_monitor_0601:
  sniffer_start_delay: 10s
```

This reduces the risk that ESP GPIO initialization influences the original Unni electronics during their own startup sequence.

After the delay, passive capture is enabled.

The shared GPIO ISR service is installed IRAM-safe before Wi-Fi initialization, while the actual sensor GPIO capture remains isolated until the startup delay has elapsed.

## Runtime recovery and diagnostics

`WiFi Home Assistant` can intentionally disable the ESPHome Wi-Fi interface.
Because Home Assistant cannot reach the device while Wi-Fi is off, connecting
USB opens a temporary recovery window in which the switch can be turned back
on. The example Wi-Fi/API configurations use `reboot_timeout: 0s` so intentional
offline operation is not treated as a fault.

During a powered CO₂ window, the passive sniffer remains active independently
of the RT/RH awake window and its PM lock keeps capture available. After the
CO₂ bus has shut down and passed the LOW/LOW blanking guards, capture is disabled
and GPIO7/SCL HIGH is explicitly armed as a Light-Sleep wake source for the next
power-up. GPIO6/SDA is never used as a wake source.

The CO₂ wake trace correlates each battery-policy cycle with one sequence
number. It logs monotonic timestamps for `power_down`, the task-side
`gpio7_wake_observed`, `capture_rearm` and the first completed capture. The wake
entry also records the ESP-IDF wake cause, GPIO wake mask and configured SCL
pin. The first capture is classified as `valid`, `valid_with_errors`,
`crc_error`, `frame_error`, `protocol_no_measurement` or `unhandled_capture`,
with frame, raw-SCL and decoder counters. Absence of a later event therefore
distinguishes a missing native power-up, an unobserved wake, a failed rearm and
a captured-but-rejected transaction without adding recovery behavior.

Debug builds periodically emit `I2C edge diag` messages as an additional way to
distinguish missing electrical activity from framing or decoder failures.

Because an intermittent passive tap can produce malformed I²C captures, decoded
CO₂ values below 350 ppm are rejected. After a CRC/framing error, publication
resumes only after two plausible consecutive readings agree within 150 ppm.

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

  unni_history_transfer_guard.h
      Unni capture-aware history transfer guard adapter

sensirion_gadget_bridge/
  __init__.py
      reusable ESPHome bridge schema and code generation

  sensirion_gadget_bridge.cpp/.h
      source binding and BLE/history lifecycle

  sensirion_bridge_core.cpp/.h
      shared profile-aware BLE/GATT/history sample and T/RH coalescer

  sensirion_ble.cpp/.h
      BLE live advertising

  sensirion_history.cpp/.h
      persistent BLE history and GATT download

  sensirion_settings.cpp/.h
      experimental Device Settings support

  sensirion_sht43_probe.cpp/.h
      experimental SHT43 identity probe

unni-smartification-c6/
  ESP32-C6 Rev A PCB work in progress
  KiCad sources, validation, mechanics and fabrication tooling

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

tests/host/
  native ESPHome host-platform smoke tests for all shipped YAML variants
```

## Native host test matrix

The repository includes a hardware-free smoke-test layer based on ESPHome's
official `host:` platform. The runner derives a temporary host configuration
from each shipped YAML variant, compiles it natively, starts the resulting host
binary directly, and waits for the portable component self-test to pass:

```sh
python3 -m pip install -r requirements-test.txt
python3 tests/host/run_host_tests.py

# Full supported ESPHome matrix (currently 2026.8.1 and 2026.8.2):
python3 tests/host/run_version_matrix.py
```

The host shim exercises the real component schema/code-generation surface, CO₂
decoder/CRC logic, calibration functions and entity wiring. It also stress-tests
a complete 4096-sample history schedule (2049 notifications: one header plus
2048 two-sample data packets) across the predictive Unni capture guard and
verifies sparse-anchor rebasing after the history ring begins overwriting old
samples. ESP32-specific GPIO,
ISR timing, ADC, Light-Sleep/PM, BLE radio/GATT and flash behavior are explicitly
out of scope and still require hardware testing. See `tests/host/README.md`.

The suite also runs the Sensirion bridge portable tests and both standalone-room-sensor YAML
variants. Its RT/RH regression corpus contains 135 timing records extracted
from the August 11 capture archives, covering normalization and portable
temperature/humidity calibration paths beyond the synthetic smoke fixture.

On macOS the runner also handles native C++20 standard-library selection. If
the active Apple libc++ lacks ESPHome's required concepts support and Homebrew
LLVM is installed, the generated host builds are automatically pointed at the
Homebrew libc++ headers and runtime; no manual `CXX=...` wrapper is required.

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
