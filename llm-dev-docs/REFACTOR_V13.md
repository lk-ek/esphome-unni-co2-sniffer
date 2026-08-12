# Refactor V13 — RT/RH decoder state cleanup

This pass deliberately keeps the decoded signal behaviour unchanged.

## Changes

- Replaced the RT/RH decoder's loose module globals with one `DecoderState`.
- Replaced loose raw-debug globals with one `DebugCaptureState`.
- Made the phase type a scoped `enum class Phase`.
- Renamed ambiguous state members (`last_any_us` -> `last_edge_us`, `last_state` -> `gpio_state`, RT-temperature accumulators expanded for readability).
- Moved the poll sequence counter into decoder state.
- Removed the obsolete, unreferenced `measurement_quality.h`; quality validation remains in `rtrh_decoder.cpp` as in V10+.

## Intentionally unchanged

- GPIO pins and ISR service model.
- REF/RT time boundaries and 15 s quiet completion rule.
- G10 falling-edge period measurement.
- RT 880-cycle temperature accumulation.
- RH characteristic-state sampling and median.
- All acceptance thresholds and quality weights.
- Calibration functions and extrapolation semantics.
- `/rt_rh_capture.csv` and `/rt_rh_timing.csv` formats and behaviour.
