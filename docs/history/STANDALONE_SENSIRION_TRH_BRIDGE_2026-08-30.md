# Standalone Sensirion T/RH bridge (2026-08-30)

`mobilesensor-sensirion.yaml` reuses the project's Sensirion/MyAmbience BLE and
persistent-history implementation on an unrelated ESPHome sensor node. The
source sensors are an AHT21 for temperature/humidity and an ENS160 for air
quality.

The new `standalone_sensirion_mode` deliberately disables all Unni-specific GPIO
capture, USB-power, battery ADC/learning and C3-only power-policy setup. External
T/RH values can be supplied through
`CO2Monitor0601::publish_external_temperature_humidity()`.

The BLE identity uses the existing SHT43 DemoBoard compatibility path (sample
type `0x06`), so no CO2 value is advertised. ENS160 `eCO2` is intentionally not
mapped to Sensirion CO2 because it is an estimated equivalent-CO2 value rather
than a direct CO2 measurement. TVOC/AQI also remain Home Assistant-only.

For the SHT43 profile, persistent history now considers a finite temperature +
humidity pair complete without requiring `have_co2`. The on-flash sample width
and existing history wire/storage format remain unchanged; unused CO2 bytes stay
zero. This avoids a flash-format migration while allowing T/RH-only sampling.

The AHT21 callbacks are coalesced by a short restartable ESPHome script before
feeding BLE, preventing a transient mixed pair when temperature and humidity
are published sequentially by the sensor component.

The existing Unni-specific mapping of Sensirion setting `0x81FE` to Wi-Fi/HA
disable is suppressed in standalone mode. The SHT43-compatible setting therefore
cannot unexpectedly disconnect this independent sensor node from Home Assistant.

Hardware/MyAmbience validation is still required on the ESP32/Wemos D1 Mini32,
especially pairing and SHT43-profile history download behavior.
