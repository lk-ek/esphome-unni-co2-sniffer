# Standalone Sensirion T/RH bridge (2026-08-30)

`mobilesensor-sensirion.yaml` reuses the project's Sensirion/MyAmbience BLE and
persistent-history implementation on an unrelated ESPHome sensor node. The
source sensors are an AHT21 for temperature/humidity and an ENS160 for air
quality. Since the 2026-08-31 extraction, the shipped mobile YAML instantiates
`sensirion_gadget_bridge` directly; the standalone mode described below remains
only as the backward-compatible predecessor.

`standalone_sensirion_mode` deliberately disables all Unni-specific GPIO
capture, USB-power, battery ADC/learning and C3-only power-policy setup. External
T/RH sources are bound with `sensirion_temperature_id` and
`sensirion_humidity_id`. The public
`CO2Monitor0601::publish_external_temperature_humidity()` method remains as a
compatibility delegate.

The original live-advertisement failure was caused by the SHT43 builder sharing
the CO2 profile's unconditional `T/RH/CO2` completeness check. The portable
`SensirionBridgeCore` now owns one profile-aware sample for advertisements,
SHT43 GATT and history. `sht43_trh` requires only finite in-range T/RH;
`trh_co2` retains the existing CO2 requirement. Invalid updates do not replace
the last valid sample, and SHT43 samples always leave CO2 absent/zero on wire.

The BLE identity uses the existing SHT43 DemoBoard compatibility path (sample
type `0x06`), so no CO2 value is advertised. ENS160 `eCO2` is intentionally not
mapped to Sensirion CO2 because it is an estimated equivalent-CO2 value rather
than a direct CO2 measurement. TVOC/AQI also remain Home Assistant-only.

For the SHT43 profile, persistent history now considers a finite temperature +
humidity pair complete without requiring `have_co2`. The on-flash sample width
and existing history wire/storage format remain unchanged; unused CO2 bytes stay
zero. This avoids a flash-format migration while allowing T/RH-only sampling.

The bridge component itself registers AHT21 state callbacks. A restartable 100 ms
coalescer commits only the newest complete pair, preventing a transient mixed
sample when ESPHome publishes temperature and humidity sequentially. The first
pair immediately starts history; later entries follow the configured interval.
The standalone history download has no producer guard. The generic transport
still owns its cursor, CCCD state and watchdogs; only an Unni composition
injects the separate capture-aware guard adapter.

The existing Unni-specific mapping of Sensirion setting `0x81FE` to Wi-Fi/HA
disable is suppressed in standalone mode. The SHT43-compatible setting therefore
cannot unexpectedly disconnect this independent sensor node from Home Assistant.

The node is treated as permanently externally powered. It configures only the
fixed 2 s advertising interval and creates no battery, VBUS or Energy Save
entities. ENS160 polling is opt-in after every boot. Only TVOC and AQI are HA
entities; eCO2 is intentionally absent everywhere. AHT21 compensation remains
active, while possible ENS160-on thermal influence is documented rather than
hidden behind an unvalidated correction.

History depends only on the configured AHT21 temperature/humidity IDs. ENS160
enable, disable, warm-up, read failure, or absence is outside the sample clock
and cannot pause or clear history. `mobilesensor-sensirion-no-ens160.yaml`
provides the fully ENS-free build: it retains the same BLE/history component
configuration and OTA flush while omitting every ENS160 driver, raw-I²C and HA
control reference.

Hardware/MyAmbience validation is still required on the ESP32/Wemos D1 Mini32,
especially pairing, live type-0x06 data, first history entry and download.
