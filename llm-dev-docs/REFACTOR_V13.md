# Refactor v13

This revision continues the readability refactor without changing protocol or
calibration behaviour.

- Re-applies the hardware-tested RT/RH `DecoderState` / `DebugCaptureState`
  grouping to the project ZIP used as the source for this revision.
- Removes the stale, unused `measurement_quality.h` file.
- Splits `BusSniffer::loop()` into orchestration plus `process_rtrh_()`,
  `process_co2_()` and `update_thermal_transient_()`.
- Replaces repetitive optional sensor publication checks with small local
  publishing helpers.
- Adds `ISR_ARCHITECTURE.md`, documenting both interrupt handlers, handoff to
  task context, critical timing assumptions and safe-change rules.

No timing thresholds, pin mappings, calibration coefficients, BLE wire formats,
history formats, or decoder validity rules were intentionally changed.
