# Native host tests

This directory contains the first hardware-free regression layer for the UNNI
external component. It uses ESPHome's official `host:` platform.

`run_host_tests.py` derives a temporary native-host configuration from each of
the seven shipped device YAML files. It removes only top-level blocks that are
ESP32/network/radio specific (`esp32`, `esp32_ble`, `wifi`, `api`, `ota`, `captive_portal`, and
`time`) and retains the real component feature blocks.
Therefore changes to a shipped variant automatically flow into its host test.
For both mobile YAMLs, the hardware-only AHT21 and optional ENS160 entries are
replaced by template sensors with the same IDs, while their real
`sensirion_gadget_bridge:` block is retained. Negative configs verify the new
bridge's paired source IDs and profile/alias conflict as well as the retained
legacy standalone-schema constraints.

The native host implementations keep the same code-generation setter surfaces
as their ESP32 implementations. At startup they:

- verifies a known-good EC05/500 ppm capture;
- verifies rejection/accounting of a bad Sensirion CRC;
- verifies the current complete-RH-burst duration envelope and its lower/upper
  rejection boundaries;
- exercises the shared RT/RH calibration functions and range invariants;
- verifies the Sensirion sample serializer against a golden byte sequence,
  finite-value checks, and encoding clamps;
- verifies profile completeness, invalid-sample retention, 100 ms T/RH
  coalescing, newest-pair selection, zeroed SHT43 history CO2 bytes, and a
  golden SHT43 type-0x06 manufacturer payload;
- verifies history interval/generation handling, sparse-time anchor rebasing
  after a full 4096-sample ring starts evicting old samples, and the portable
  predictive, reactive, adaptive-quality, stale-frame, overlap, and wrap-safe
  download-guard boundaries;
- simulates a full 4096-sample MyAmbience schedule (2049 notifications) at the
  production 4 ms pacing while crossing normal six-second CO2 guard windows;
- publishes deterministic fixture values to any entities enabled by that YAML
  variant;
- prints `UNNI HOST SELF-TEST PASSED` for Unni variants or
  `SENSIRION BRIDGE HOST SELF-TEST PASSED` for standalone mobile variants and
  marks the corresponding component failed otherwise.

It intentionally does **not** emulate GPIO edges, ISR timing, ADC, light sleep,
ESP power-management locks, BLE radio/GATT behavior, Wi-Fi, or flash partition
semantics. Those remain hardware-in-the-loop concerns.

## Run

With the current tested ESPHome release installed (`requirements-test.txt` pins 2026.8.2):

```sh
python3 tests/host/run_host_tests.py
```


To exercise the complete supported-version matrix in isolated virtual environments:

```sh
python3 tests/host/run_version_matrix.py
```

The matrix currently covers ESPHome 2026.8.1 (minimum supported) and 2026.8.2
(current project pin). `--mode config` and `--mode compile` are forwarded to the
normal runner. `--find-links /path/to/wheelhouse` enables a fully offline run
when matching wheels for both versions are available.

The runner first looks for an `esphome` console script next to the Python
interpreter executing the test harness (for example `/opt/esphome/bin/esphome`
when invoked as `/opt/esphome/bin/python ...`). It falls back to `PATH` only if
no interpreter-adjacent executable exists. `--esphome /path/to/esphome` remains
available as an explicit override.

The default mode first runs `esphome compile`, then starts the resulting native
binary directly and terminates it as soon as the self-test success marker is
observed. It deliberately does not use `esphome run`, because host targets are
long-lived processes and `run` is intended for interactive foreground use.

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

## Native compiler requirement

ESPHome 2026.7.x host builds require C++20, including standard-library
features such as concepts and `std::span`. The runner performs a native
compiler + standard-library preflight before building the matrix and injects
the exact tested C++ flags into every generated host configuration.

On macOS the compiler binary and libc++ are treated separately. Some Apple
Command Line Tools combinations accept `-std=gnu++20` while their selected
libc++ does not expose the C++20 concepts required by ESPHome. When Homebrew
LLVM is installed, the runner automatically tries the system compiler against
Homebrew LLVM's libc++ headers/runtime using `-nostdinc++`, an explicit libc++
include path, library path, and runtime rpath. This matches PlatformIO
`native` more reliably than setting `CXX` alone, because PlatformIO may still
invoke the system compiler internally.

If Homebrew LLVM is unavailable or unsuitable, the runner falls back to the
default native toolchain. If neither combination passes the same C++20 probe
used for the generated build flags, the matrix stops with a focused diagnostic.

## Host/ESP32 header boundary

ESPHome's generated `esphome.h` includes component headers even when their ESP32-only `.cpp` files are filtered out. Public component headers therefore remain parseable on `host:`: ESP-IDF-only includes live behind platform guards, while the native shim supplies the codegen-facing component API. This keeps the host matrix useful without pretending to emulate GPIO/ISR/PM/BLE hardware.


## Archived RT/RH capture regression

The runtime smoke test also loads `tests/host/fixtures/rtrh_legacy_260811.csv`.
It contains 135 normalized timing records extracted from the existing
`rh_th_260811*.zip` and `rh_th_nod3_260811-1.zip` capture sets. The host test
checks the measured count/period/duration relationships, reconstructs RT/REF and
RH-carrier/REF ratios from those measurements, and runs every recorded point
through the production calibration functions. Golden engineering-unit outputs
are intentionally frozen in the fixture, so a calibration change is visible as
an explicit regression that must be reviewed and accepted.

The archived 2026-08-11 captures predate the final two-wire/10 kOhm sensor
loading and therefore are not treated as current physical calibration truth.
They are used as a broad real-world timing/input corpus. Current calibration
coefficients remain documented separately under `docs/history/`.
