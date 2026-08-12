# Final code/config audit

This audit was performed after the decoder/BLE/history refactors. Its purpose is to document what was intentionally left alone and which clearly stale pieces were removed.

## Production code

No timing thresholds, ISR logic, calibration equations, BLE protocol bytes, history wire format, or flash metadata format were changed in this cleanup pass.

The current responsibility split is:

- `bus_sniffer.cpp/.h`: orchestration and ESPHome-facing state
- `co2_decoder.cpp/.h`: passive CO₂ capture/decode
- `rtrh_decoder.cpp/.h`: RT/RH capture, validation, quality, calibrated result
- `sensirion_sample.h`: common BLE/history sample encoding
- `sensirion_ble.cpp/.h`: live BLE and GAP/GATT connection handling
- `sensirion_history.cpp/.h`: RAM/flash history and history download

## Removed stale configuration

- Removed global YAML `esphome.includes: <esp_mac.h>` from all build variants. `sensirion_ble.cpp` already includes the header locally; the no-BLE build does not need it at all.
- Removed the `senshist` flash partition from the no-BLE YAML.
- Removed the meaningless BLE advertising interval from the no-BLE YAML.
- Corrected the no-BLE comments so they no longer refer to BLE blocks that are not present.
- Aligned the no-BLE build with production defaults for `debug_metrics` and `sniffer_start_delay`.

## Repository cleanup

- Current ISR documentation moved to `docs/ISR_ARCHITECTURE.md`.
- Dated development/refactor notes moved to `docs/history/` and explicitly marked historical.
- Capture/VCD and macOS BLE inspection utilities moved to `tools/`.
- Python cache artifacts were removed.

## Intentionally retained

The explicit ESPHome C++ setter methods remain. Replacing them with generic integer/enum output slots would reduce a few lines but make the Python/C++ code-generation boundary harder to understand and easier to break.

Likewise, diagnostic outputs remain individually configurable in YAML. This keeps production entities optional without coupling them to `debug_metrics`.

## Source of truth

For current behavior, use:

1. `README.md`
2. `docs/ISR_ARCHITECTURE.md`
3. the source under `bus_sniffer/`

`docs/history/` is rationale/history only.

## BLE server initialization ordering fix (2026-08-12)

The component-managed YAML refactor initially called `BLEServer::create_service()`
from a generated setter while ESPHome code generation was still wiring the
BLE server. In ESPHome 2026.7.4 the server parent is assigned later by
`BLEServer::set_parent()`, while `create_service()` may immediately access the
parent when deciding whether to create a live service.

The component now stores the generated BLE server pointer only. Device
Information customization and Sensirion GATT service creation are deferred to
`BusSniffer::setup()`, after all generated object wiring has completed.
