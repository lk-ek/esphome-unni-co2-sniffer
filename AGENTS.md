# Project guide for coding agents

## Purpose

This repository smartifies an Unni CO2 monitor while preserving the original
electronics. The production firmware runs as an ESPHome external component on a
Seeed Studio XIAO ESP32-C3. It passively decodes the existing CO2 I2C traffic and
the RT/RH timing signals, publishes measurements to Home Assistant, and can
provide Sensirion/MyAmbience-compatible BLE live data and persistent history.

The `unni-smartification-c6/` subtree is a separate ESP32-C6 Rev A hardware
design in progress. Do not confuse its PCB/power design with the currently
tested XIAO ESP32-C3 installation.

## Sources of truth

Use current information in this order:

1. The implementation under `co2_monitor_0601/` and the shipped YAML variants.
2. `README.md` for the supported user-facing design and configuration.
3. `docs/ISR_ARCHITECTURE.md` for ISR, concurrency, and decoder invariants.
4. `tests/host/README.md` and the tests themselves for the native regression
   contract.
5. `docs/DEVELOPMENT_HISTORY.md`, `docs/LIGHT_SLEEP.md`, and `docs/history/`
   for rationale and historical experiments only.

Historical notes may describe superseded pin arrangements, calibration data,
power policies, advertising intervals, debug transports, or experiments. Do
not restore an older behavior merely because it is documented there. When
documentation and code disagree, inspect recent history and tests, then treat
the current code as authoritative unless the task explicitly changes it.

## Hardware and safety invariants

- The firmware is primarily a passive sniffer. Never drive the original Unni
  CO2, RT, or RH signal paths in normal operation.
- The tested installation uses a 10 kohm series resistor in every passive ESP
  tap. The resistor belongs in the ESP branch, never in the original Unni path.
- Current XIAO ESP32-C3 defaults are RT GPIO3, RH GPIO4, CO2 SDA GPIO6, CO2 SCL
  GPIO7, battery ADC GPIO2, and VBUS detection GPIO5. Configured GPIOs must be
  unique, and the battery input must be ADC1-capable.
- Do not enable internal pulls on passively observed Unni lines.
- `active_i2c_probe` is an explicitly experimental exception. It must remain
  disabled by default and is safe to consider only with the documented series
  resistance and idle-bus guards.
- RT/RH calibration depends on the physical tap arrangement. Do not mix the
  final two-wire/10 kohm calibration with older direct or three/four-wire data.
- Battery state of charge is intentionally unavailable while physical USB is
  present. Charger-influenced voltage is not an open-circuit SOC measurement.
- The learned battery model estimates effective runtime, not battery capacity
  in mAh; there is no coulomb counter.

## Architecture

Keep responsibilities separated:

- `co2_monitor_0601.cpp/.h`: orchestration, ESPHome-facing state, power policy,
  publication, and subsystem coordination.
- `i2c_sniffer.cpp/.h`: generic passive I2C edge capture and frame assembly.
- `co2_decoder.cpp/.h`: CO2 command/response and CRC decoding.
- `rtrh_decoder.cpp/.h`: RT/RH capture, validation, quality, and calibration.
- `sensirion_sample.h`: shared BLE/history sample representation.
- `sensirion_ble.cpp/.h`: advertising, GAP/GATT behavior, and live data.
- `sensirion_history.cpp/.h`: persistent sample ring and history download.
- `__init__.py`: ESPHome schema, validation, code generation, build flags,
  sdkconfig, and partition requirements.

Preserve the Python-to-C++ setter surface unless a task deliberately changes
the code-generation API. The public headers must remain parseable for ESPHome's
`host:` platform even when ESP32-only implementation files are excluded.

## ISR and concurrency rules

Read `docs/ISR_ARCHITECTURE.md` completely before changing GPIO interrupts,
edge capture, wake handling, decoder handoff, or ISR-shared state.

- ISR work must remain bounded and IRAM-safe. Do not allocate, log, use network
  APIs, touch flash, publish ESPHome state, or perform floating-point work in an
  ISR.
- Keep expensive parsing, calibration, publication, BLE/history work, and debug
  transport in task/loop context.
- Preserve critical sections, volatile/atomic semantics, snapshot handoff, and
  interrupt masking around shared state and power-management lock transitions.
- Preserve initial pin-level capture and both-pin I2C edge observation; decoder
  correctness depends on reconstructing the bus state around each edge.
- Missing-clock recovery is deliberately bounded and protocol-validated. Do
  not reintroduce the retired RMT approach or invent arbitrary edges.
- RT/RH phase selection is time-based. Only the physical RT interrupt measures
  RT periods, and measurement finalization occurs after silence in normal
  context rather than inside the ISR.
- Debug instrumentation must not materially perturb capture timing.

## Power policy

- Physical VBUS and the effective power policy are distinct. `Energy Save
  Mode` emulates battery policy while `USB Power` continues to report reality.
- USB policy favors responsiveness: no automatic Light Sleep, CPU held at the
  tested maximum policy frequency, continuous CO2 capture, immediate
  publication, and the USB BLE interval.
- Battery policy favors low average power: automatic Light Sleep, event-driven
  RT/RH and CO2 wake windows, bounded awake failsafe, throttled HA publication,
  and the battery BLE interval.
- The CO2 bus has a real powered-window lifecycle. Preserve capture gating
  before rail collapse, quiet LOW/LOW qualification, guard time, passive SCL
  HIGH wake arming, and capture restoration on the next power-up.
- Power-management locks can be acquired from ISR context but must not race
  with task-context release. Preserve the documented synchronization.
- Paired PM locks have per-handle ownership. On partial acquire, roll back the
  acquired handle; on partial release, retain ownership of the unreleased
  handle. Never replace this state with one aggregate Boolean.
- Wi-Fi, BLE, timers, logging, and debug clients can reduce Light Sleep
  residency. Do not interpret configured Light Sleep as proof of actual sleep.
- OTA start must flush/checkpoint persistent battery-learning state and pending
  BLE history as implemented.

## Measurements and publication

- Keep diagnostic RT-model temperature, Unni display-emulation values, and the
  externally calibrated physical-air values conceptually separate.
- Production `Temperature` is the physical-air estimate in its validated
  envelope, with the documented RT-model fallback. Production `Humidity` is
  carrier-based physical RH. Display-emulation values are diagnostics and must
  not silently become BLE physical measurements.
- Preserve plausibility checks, structured reject reasons, calibration-envelope
  flags, extrapolation behavior, and reject-safe retention rules.
- Production YAML intentionally creates a small entity set. Debug-only metrics
  must remain optional and absent from the normal build.
- A valid decoder result, Home Assistant publication, BLE advertising, and
  persistent-history storage are different operations with different timing.
  Avoid coupling them unnecessarily.

## BLE and persistent history

- The normal identity and wire behavior intentionally target Sensirion
  Gadget/MyAmbience compatibility without claiming Sensirion manufactured the
  device. Preserve third-party attribution in `THIRD_PARTY_NOTICES.md` and
  `LICENSES/`.
- Compatibility-sensitive GAP/local identity, Device Information fields,
  manufacturer payload layout, GATT UUIDs, CRCs, history wire format, and flash
  metadata must not change casually.
- `ble_device_name` controls the model/alternative name, not every
  compatibility-sensitive identity field.
- Do not refresh/reconfigure advertising while a GATT client is connected.
  Restore the current manufacturer payload deterministically after disconnect.
- BLE privacy, settings, pairing, and the SHT43 identity probe are experimental.
  Keep production defaults conservative.
- History is a flash-backed 4096-sample ring with a small pending RAM queue.
  Avoid large in-RAM duplication and preserve persistence compatibility unless
  an explicit migration is designed and tested.
- History metadata V3 is an A/B journal in 4-KiB erase sectors 0 and 15 of the
  64-KiB partition; data sectors 1..14 and the sample wire format remain stable.
  Keep V2 read/migration support, wrap-safe generation comparison, and retry
  dirty metadata until checkpointed.
- History intervals are limited to 60 seconds through 24 hours. Changing one
  queues asynchronous clearing: pause sampling/download and erase at most one
  sector per loop. BLE callbacks must not erase flash or synchronize preferences.
- `ble_identity_mode: legacy_fixed` is the compatibility default. Treat
  `device_derived` as an explicit migration because it can invalidate bonds and
  MyAmbience caches.
- `runtime_diagnostics` is off in production and compiled out. Keep fast-loop
  capture completion and silence detection independent of housekeeping deadlines.
- Debug UDP capture may experience lwIP `ENOMEM`; bounded retry/drop behavior
  and the one-client native API limit in debug builds protect heap availability.

## Shipped build variants

- `i2c-sniffer.yaml`: production Wi-Fi + Home Assistant + BLE.
- `i2c-sniffer-debug.yaml`: diagnostics and raw capture.
- `i2c-sniffer-no-ble.yaml`: Wi-Fi/HA baseline without BLE.
- `i2c-sniffer-ble-only.yaml`: BLE power-measurement build without Wi-Fi, API,
  captive portal, OTA, or HA exposure.
- `i2c-sniffer-sht43-probe.yaml`: experimental reverse-engineering identity;
  never treat it as the normal device configuration.

Keep all variants valid when changing the schema or generated setters. BLE-off
must compile without ESP-IDF Bluetooth headers, and host code must compile
without ESP-IDF-only public-header dependencies.

## Validation

Run validation proportional to the change:

```sh
python3 tests/host/run_host_tests.py --mode config
python3 tests/host/run_host_tests.py --mode compile
python3 tests/host/run_host_tests.py
```

The full host runner derives configurations from all five shipped YAML files,
compiles them, runs the native self-test, and checks the archived RT/RH corpus.
It does not validate GPIO timing, ISR behavior, ADC, Light Sleep, BLE radio/GATT,
Wi-Fi, or flash partition semantics; hardware-in-the-loop evidence is required
for those areas.

For targeted ESPHome checks, compile the affected shipped YAML configuration.
Do not claim hardware validation from a successful host build.

For `unni-smartification-c6/`, use its documented validators and generated
artifacts rather than a separately redrawn circuit. Important entry points are
the direct-from-KiCad ngspice validation, the worst-case battery sweep, the
mechanical/connectivity checks, and the stable KiCad 10.0.5 compatibility/export
flow. KiCad 10.99 sources are canonical; the `dist/kicad-10.0.5/` tree is a
generated, disposable compatibility copy and must not be edited as a second
source of truth.

## Change discipline

- Inspect the relevant implementation, current documentation, tests, and Git
  diff before editing. Preserve unrelated user changes in a dirty worktree.
- Prefer focused changes; do not combine calibration, protocol, persistence,
  ISR, or power-policy changes without a clear reason.
- Never change timing thresholds, calibration coefficients, protocol bytes,
  flash formats, PCB power-path behavior, or safety guards as cleanup.
- Update `README.md` and current architecture documentation when supported
  behavior changes. Put dated experimental rationale in `docs/history/` only
  when it remains useful.
- Keep production logs bounded. High-rate diagnostics belong behind debug
  options and must account for timing, heap, and network backpressure.
- Before handing off, report exactly what was tested and what still requires
  device or PCB validation.

## ESP32-C6 hardware-design constraints

When working below `unni-smartification-c6/`, read its `README.md`,
`DESIGN_DECISIONS.md`, `MECHANICAL.md`, and `SIMULATION.md` first.

- The original Unni connections remain T-taps and usable without the ESP.
- External USB VBUS and internally generated `UNNI_AC` are distinct rails.
- Preserve autonomous MCP73831 charging, source isolation, USB hardware inhibit
  of the 5 V boost, VBUS pull-down, and backfeed prevention.
- ESP32-C6 strapping pins and the RF keepout are hard constraints. The antenna
  region must remain copper-free on all four layers and clear of nearby metal.
- Preserve the measured USB-board datum, mounting-hole coordinates, enclosure
  clearance, B-side placement keepout, and board growth direction.
- Both inner layers are intended as ground reference planes. Keep switch nodes
  compact and respect the documented PCBWay-compatible fabrication rules.
- Simulation models are system-level behavioral checks, not proof of ripple,
  stability, EMI, thermal limits, inductor saturation, or switch-current margin.
