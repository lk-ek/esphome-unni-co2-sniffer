# BLE compile-time options

This version is based on the modular source tree from `esphome-i2c-sniffer-ble.zip`:

- `bus_sniffer.cpp/.h`
- `sensirion_ble.cpp/.h`
- `sensirion_history.cpp/.h`

No RT/RH calibration constants were changed.

## YAML options

```yaml
bus_sniffer:
  ble: true
  ble_live: true
  ble_history: true
  ble_id: unni_ble
  ble_server_id: unni_ble_server
```

The component emits these compile-time defines:

```cpp
UNNI_BLE_ENABLED
UNNI_BLE_LIVE_ENABLED
UNNI_BLE_HISTORY_ENABLED
```

`ble_options.h` provides defaults and dependency checks.

## Test builds

### A: Current/full BLE

```yaml
ble: true
ble_live: true
ble_history: true
```

### B: BLE live, no history

```yaml
ble: true
ble_live: true
ble_history: false
```

This completely compiles `sensirion_history.cpp` out.

### C: BLE stack present, no live updates, no history

```yaml
ble: true
ble_live: false
ble_history: false
```

The ESPHome BLE controller/server still runs, but `bus_sniffer` does not update
Sensirion T/RH/CO2 advertisements and has no history code.

### D: Genuine no-BLE build

Use `i2c-sniffer-no-ble.yaml`, or set:

```yaml
bus_sniffer:
  ble: false
  ble_live: false
  ble_history: false
```

and remove the top-level `esp32_ble:` and `esp32_ble_server:` blocks.
No `ble_id`/`ble_server_id` is required in this configuration.

In this build `sensirion_ble.cpp`, `sensirion_history.cpp`, all BLE event handlers,
and BLE server types in `bus_sniffer.h` are compiled out.

## Suggested A/B order

A -> B -> C -> D, with 3-5 RT/RH measurements per build and unchanged calibration.
Compare the raw `RT/REF` and `RH-state/REF` ratios in addition to the converted values.
