# Unni CO₂ Sensor Smartification

This project turns an otherwise standalone Unni CO₂ monitor into a networked sensor without replacing its original electronics.

A Seeed Studio XIAO ESP32-C3 is added as a **passive sniffer**. It observes the existing CO₂ and temperature/humidity measurement signals, decodes them, and makes the readings available through:

- ESPHome and Home Assistant
- Bluetooth Low Energy advertisements compatible with the Sensirion gadget format
- a Sensirion-style GATT history service for history downloads in MyAmbience

The original Unni MCU, display, buttons, alarms, and measurement circuitry remain in place.

This is a reverse-engineering project. The signal interpretation and calibration were derived experimentally and may not apply unchanged to other Unni hardware revisions.

## Features

- Passive CO₂ bus sniffing; the ESP does not act as the bus master
- Passive decoding of the original temperature/humidity measurement timing
- Temperature-compensated relative-humidity conversion
- Measurement-quality and calibration-range diagnostics
- Native ESPHome API for Home Assistant
- Sensirion-compatible BLE live advertising
- Sensirion-style BLE history storage and GATT download
- Persistent BLE history in a dedicated flash partition
- Configurable BLE, history, debug, and publish features at compile time
- Low-power-oriented production configuration: 80 MHz CPU, Wi-Fi power saving, slow BLE advertising, and optional removal of debug capture infrastructure
- GPL-3.0-or-later licensed

## Hardware

### Required parts

- Unni CO₂ monitor
- [Seeed Studio XIAO ESP32-C3](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html)
- Fine insulated wire suitable for PCB work
- Soldering iron and fine solder
- Ideally 4.7 kΩ to 10 kΩ series resistors for each sniffed signal line
- A USB power source capable of powering both the Unni monitor and the XIAO ESP32-C3

The ESP and the Unni electronics share **+5 V and GND**. The ESP only observes the signal lines.

### Signal connections

The current firmware uses the following XIAO ESP32-C3 pins:

| XIAO pin | ESP32-C3 GPIO | Unni signal | Purpose |
|---|---:|---|---|
| D5 | GPIO7 | CO₂ SCL | Clock of the CO₂ digital bus |
| D4 | GPIO6 | CO₂ SDA | Data of the CO₂ digital bus |
| D1 | GPIO3 | RT/RH G10 | RT/RH timing signal |
| D3 | GPIO5 | RT/RH G11 | RT/RH state/timing signal |
| D2 | GPIO4 | RT/RH G13 | RT/RH state/timing signal |
| 5V | — | +5 V | Shared power |
| GND | — | GND | Shared ground |

The former G12 signal was found to duplicate G10 and is not required by the current decoder.

> **Image placeholder — Unni PCB overview**  
> Add a photo showing the Unni main PCB, the CO₂ sensor board, the RT/RH sensor area, and the XIAO installation location.

> **Image placeholder — signal test points**  
> Add a close-up with the five sniffed signal pads labelled: CO₂ SCL, CO₂ SDA, G10, G11, and G13.

> **Image placeholder — XIAO wiring**  
> Add a photo or diagram showing D1/D2/D3/D4/D5, +5 V, GND, and the recommended series resistors.

### Recommended series resistors

For a permanent installation, placing approximately **10 kΩ in series with every signal connection to the XIAO** is recommended:

```text
Unni signal ---- 10 kΩ ---- XIAO GPIO
```

The resistor belongs only in the ESP sniffing branch; do not insert it into the original Unni signal path.

The ESP inputs are intended to remain high impedance. Series resistance further reduces the chance that ESP boot states, protection diodes, accidental pin configuration, or power-up sequencing can influence the original Unni electronics. The measured signal periods are long enough that 10 kΩ is not expected to be problematic for this passive input application.

### Power and thermal considerations

The production configuration runs the ESP32-C3 at 80 MHz and disables the heavy capture/debug infrastructure when it is not needed. This substantially reduces power consumption and ESP self-heating, which is desirable because the added board is physically close to temperature-sensitive circuitry.

The current configuration also supports a delayed sniffer initialization. During `sniffer_start_delay`, the custom component does not configure or read the five signal GPIOs. This is primarily a boot-isolation measure; Wi-Fi and BLE can initialize normally in parallel.

## Software layout

There are two related but distinct parts in this repository:

```text
i2c-sniffer.yaml          ESPHome device project / firmware configuration
bus_sniffer/              reusable ESPHome external component
```

### The ESPHome project

`i2c-sniffer.yaml` describes one complete firmware image for the XIAO ESP32-C3. It contains the normal ESPHome configuration such as:

- board and ESP-IDF framework selection
- Wi-Fi
- native Home Assistant API
- OTA updates
- BLE controller/server configuration
- flash partition for Sensirion-compatible history
- the `bus_sniffer:` component instance and its user-facing options
- the sensors and diagnostic entities that should be exposed

This is the file you normally copy, edit, compile, and flash for a specific device.

The project currently loads the component from the repository itself:

```yaml
external_components:
  - source:
      type: local
      path: .
```

ESPHome calls this mechanism an **External Component**. See the ESPHome documentation:  
<https://esphome.io/components/external_components/>

### The `bus_sniffer` ESPHome component

`bus_sniffer/` contains the reusable implementation. It is not a standalone firmware image.

The directory contains both the ESPHome/Python integration layer and the C++ implementation:

```text
bus_sniffer/
  __init__.py              YAML schema and ESPHome code generation
  bus_sniffer.cpp/.h       thin ESPHome/HA/BLE orchestration layer
  co2_decoder.cpp/.h       GPIO capture + passive CO₂/I²C decoding
  rtrh_decoder.cpp/.h      RT/RH GPIO ISR, phase capture and raw snapshots
  calibration.h            temperature/RH conversion coefficients and ranges
  measurement_quality.h    measurement-quality logic
  sensirion_ble.cpp/.h     Sensirion-compatible live BLE advertising
  sensirion_history.cpp/.h Sensirion-style GATT history and persistence
  ble_options.h            compile-time BLE feature selection
```

This split is intentional. `BusSniffer` is now deliberately small: it starts the decoders, converts completed RT/RH snapshots into calibrated values, publishes ESPHome entities, and forwards valid measurements to BLE. The timing-critical GPIO/ISR code lives behind the two decoder APIs, so changes to Home Assistant or BLE no longer require editing the protocol decoders.

A second example, `i2c-sniffer-no-ble.yaml`, demonstrates a genuine no-BLE build.

## ESPHome configuration

A reduced production-oriented configuration looks roughly like this:

```yaml
esp32:
  board: seeed_xiao_esp32c3
  cpu_frequency: 80MHz
  framework:
    type: esp-idf

external_components:
  - source:
      type: local
      path: .

bus_sniffer:
  ble: true
  ble_live: true
  ble_history: true

  ble_advertising_interval: 2s
  ha_publish_interval: 30s
  sniffer_start_delay: 10s

  debug_capture: false
  debug_metrics: false

  co2:
    name: "CO2"
  rt_temperature:
    name: "RT Temperature"
  rh_humidity:
    name: "RH Humidity"
```

See `i2c-sniffer.yaml` for the complete configuration, including BLE server IDs, API encryption, OTA, Wi-Fi settings, history partitioning, and optional diagnostic sensors.

### Debug capture

`debug_capture` is a compile-time option.

```yaml
bus_sniffer:
  debug_capture: false
```

With `debug_capture: false`, the raw-capture archive, RT/RH debug buffers, CSV HTTP handlers, and related HTTP debug code are not compiled into the component. A `web_server:` block is therefore not required in the normal production build.

When developing or reverse-engineering signals, enable it together with an ESPHome web server:

```yaml
web_server:
  port: 80

bus_sniffer:
  debug_capture: true
```

## Home Assistant

The normal network interface is the **ESPHome native API**. Home Assistant can discover or add the device through its ESPHome integration and expose the configured sensor entities directly. Home Assistant documents the integration here:  
<https://www.home-assistant.io/integrations/esphome/>

The primary entities are:

- CO₂ concentration in ppm
- RT-derived temperature
- RH-derived relative humidity

Optional diagnostic entities include:

- reference period
- RT period and RT/reference ratio
- RH state period and RH/reference ratio
- logarithmic RH ratio
- measurement quality
- thermal-transient state
- temperature/RH/calibration extrapolation flags
- CO₂ CRC and frame error counters

`ha_publish_interval` controls how often decoded values are published to Home Assistant. The production configuration uses 30 seconds because Home Assistant generally does not need every raw measurement cycle.

The BLE path is independent of Home Assistant. Wi-Fi/API and BLE can therefore be used at the same time.

## Sensirion-compatible Bluetooth support

The firmware implements its BLE support using ESPHome's ESP-IDF Bluetooth stack. It does **not** embed NimBLE-Arduino or the Sensirion Arduino library at runtime.

The over-the-air format was developed against Sensirion's public Gadget BLE examples and protocol behavior. The current live advertisement uses the Sensirion company identifier and the `T_RH_CO2_ALT`/MyCO2-style sample layout for temperature, relative humidity, and CO₂.

Useful Sensirion references:

- Sensirion Gadget BLE library: <https://github.com/Sensirion/arduino-ble-gadget>
- Sensirion demonstrators and MyAmbience: <https://sensirion.com/products/demonstrators-and-apps>
- Sensirion SCD4x BLE Gadget resources: <https://sensirion.com/resource/software/ble-gadget/scd4x>

### Live advertisements

Live temperature, humidity, and CO₂ values are encoded into Sensirion-compatible legacy BLE manufacturer data. Advertising is deliberately slower than ESPHome's normal connectable advertising in order to reduce RF duty cycle and power consumption.

The default in this project is:

```yaml
ble_advertising_interval: 2s
```

### GATT history

The component also implements a Sensirion-style GATT history service. Samples are retained in RAM and persisted in a dedicated flash partition. MyAmbience can connect to the gadget and download the history.

Advertising refreshes are paused while a GATT client is connected. This avoids a race between advertisement reconfiguration and the active history connection, which otherwise could leave MyAmbience stuck at "connecting to gadget" or interrupt downloads.

### Sensirion MyAmbience

[Sensirion MyAmbience](https://sensirion.com/products/demonstrators-and-apps) is the primary compatibility target for the BLE implementation. Sensirion describes MyAmbience as its app for viewing Bluetooth-enabled demonstrators and downloading historical data from devices that support it.

Current official downloads are linked from Sensirion's page above for both iOS and Android.

### "CO2 sensor" app

The open-source **CO2 sensor** app by Simon Loffler is another useful BLE client for Sensirion-style CO₂ gadgets. It is available for iPhone, iPad, and macOS and is designed to read CO₂, temperature, and humidity from Bluetooth Sensirion SCD41-style devices.

- App Store: <https://apps.apple.com/app/co2-sensor/id1643286074>
- Source code: <https://github.com/sighmon/ios-ble-co2-sensor>

Application behavior may differ because this project emulates the relevant BLE behavior rather than containing a genuine Sensirion SCD41.

## Temperature and humidity decoding

The Unni unit does not expose temperature and humidity as a straightforward digital register on the sniffed lines. The firmware measures timing relationships between the observed RT/RH signals and derives normalized ratios.

Temperature is derived from the RT/reference timing ratio. Relative humidity uses a temperature-compensated logarithmic model based on the RH/reference ratio.

The current coefficients are stored in `bus_sniffer/calibration.h`.

Calibration has been derived experimentally from captured measurements. Recent testing with an independent BME280 placed next to the Unni showed the decoded ESP temperature and humidity to be much closer to the local reference than the Unni display during the Unni's post-boot warm-up period. The Unni display itself therefore should not automatically be treated as ground truth, especially shortly after power-up.

Diagnostic extrapolation flags indicate when a measurement lies outside the range covered by the calibration data.

## CO₂ decoding

The CO₂ path passively observes the existing clock/data traffic between the Unni electronics and its CO₂ sensor circuitry. Frames are reconstructed from edges, validated, and decoded without driving the original bus.

CRC and frame-error counters are available as optional ESPHome diagnostic sensors.

## Building and flashing

Install ESPHome using your preferred supported method, create a `secrets.yaml` containing the secrets referenced by `i2c-sniffer.yaml`, and build from the repository directory.

For example:

```bash
esphome compile i2c-sniffer.yaml
esphome run i2c-sniffer.yaml
```

After the first installation, OTA updates can be used normally.

For ESPHome documentation, see:  
<https://esphome.io/>

## Development notes

The repository includes several dated calibration and development notes documenting the reverse-engineering process, BLE experiments, measurement-quality work, power-saving changes, and history-download fixes. They are useful when changing the low-level decoder, but are not required for a normal production build.

If you change the hardware revision, pin assignment, calibration, or RT/RH sensor circuitry, verify the raw ratios and diagnostic quality metrics before trusting the converted values.

## Safety and warranty

This modification requires opening the device and soldering to its electronics. It can damage the monitor, the ESP, or the attached computer/power supply if wired incorrectly. Verify voltage levels and pin assignments on your own hardware before connecting the XIAO.

This project is not affiliated with or endorsed by the manufacturer of the Unni monitor or by Sensirion. Sensirion names and protocol references are used only to describe compatibility with their published gadget ecosystem.

## License

This project is licensed under **GNU General Public License v3.0 or later (`GPL-3.0-or-later`)**. See [LICENSE](LICENSE).

Third-party libraries, tools, applications, trademarks, and documentation remain subject to their respective licenses and owners.
