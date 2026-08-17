# Native host tests

This directory contains the first hardware-free regression layer for the UNNI
external component. It uses ESPHome's official `host:` platform.

`run_host_tests.py` derives a temporary native-host configuration from each of
the five shipped device YAML files. It removes only top-level blocks that are
ESP32/network/radio specific (`esp32`, `esp32_ble`, `wifi`, `api`, `ota`, and
`captive_portal`) and retains the real `co2_monitor_0601:` feature block.
Therefore changes to a shipped variant automatically flow into its host test.

The component's native host implementation keeps the same code-generation
setter surface as the ESP32 implementation. At startup it:

- verifies a known-good EC05/500 ppm capture;
- verifies rejection/accounting of a bad Sensirion CRC;
- exercises the shared RT/RH calibration functions and range invariants;
- publishes deterministic fixture values to any entities enabled by that YAML
  variant;
- prints `UNNI HOST SELF-TEST PASSED` on success and marks the component failed
  otherwise.

It intentionally does **not** emulate GPIO edges, ISR timing, ADC, light sleep,
ESP power-management locks, BLE radio/GATT behavior, Wi-Fi, or flash partition
semantics. Those remain hardware-in-the-loop concerns.

## Run

With ESPHome 2026.7.4 (or a newer compatible release) installed:

```sh
python3 tests/host/run_host_tests.py
```

The default mode performs a native compile and starts every resulting host
binary long enough to observe the self-test success marker.

Faster validation-only modes are also available:

```sh
python3 tests/host/run_host_tests.py --mode config
python3 tests/host/run_host_tests.py --mode compile
```

Temporary `.host-test-*.yaml` files are generated in the repository root so the
existing `external_components: path: .` remains valid. They are removed after
the run unless `--keep-generated` is supplied.

Host preferences are isolated below `.esphome/host-test-prefs` using
`ESPHOME_PREFDIR`.
