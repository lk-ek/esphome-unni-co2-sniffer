# Config/codegen refactor

The ESPHome YAML API is intentionally unchanged.

## Python side

`bus_sniffer/__init__.py` now has two declarative output tables:

- `SENSOR_OUTPUTS`
- `BINARY_OUTPUTS`

Each normal sensor is described once as `(schema, setter)`. Both `CONFIG_SCHEMA`
and `to_code()` are generated from those tables, so adding or removing an
optional diagnostic entity no longer requires editing two long independent
blocks.

`co2` remains required; every other entity remains optional.

## C++ side

Runtime members are grouped by responsibility:

- `Outputs out_` — ESPHome sensor/binary-sensor pointers
- `HaState ha_` — throttled Home Assistant publishing
- `ThermalState thermal_` — temperature-rate hysteresis
- `Co2State co2_` — last value and error counters

The small explicit setter methods remain on purpose. ESPHome codegen can keep
calling the same stable methods as before, while the runtime implementation no
longer exposes dozens of unrelated member variables.

## Compatibility

No YAML keys or defaults were changed. BLE feature validation and compile-time
defines are unchanged.
