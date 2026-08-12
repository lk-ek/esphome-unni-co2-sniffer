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
