# Standalone binary-sensor codegen guard (2026-08-30)

`mobilesensor-sensirion.yaml` uses the shared `co2_monitor_0601` runtime in
`standalone_sensirion_mode`. In this mode the component deliberately removes
all Unni-specific binary-sensor outputs, while the shared C++ declarations
still reference ESPHome's `BinarySensor` type.

With ESPHome 2026.8.1 this produced a compile failure because
`USE_BINARY_SENSOR` was enabled but no `ESPHOME_ENTITY_BINARY_SENSOR_COUNT`
macro was generated.

The standalone device configuration now contains ESPHome's normal `status`
binary sensor. This keeps the domain instantiated through ESPHome's own entity
codegen and also exposes a useful online/offline entity to Home Assistant.
