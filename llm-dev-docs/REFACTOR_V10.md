# Refactor V10 — RT/RH decoder owns complete measurements

The RT/RH component boundary now follows the data flow rather than the historical reverse-engineering steps.

`rtrh_decoder` owns:

- GPIO edge capture and fixed-time REF/RT/RH phase decoding
- timing aggregation and RH-state median
- capture acceptance/reject rules
- measurement quality score
- RT/RH ratios and logarithmic RH ratio
- temperature/humidity calibration
- calibration extrapolation flags

Its public output is one `rtrh_decoder::Measurement`. Internal ISR accumulators, snapshots, and median buffers are no longer exposed in the header.

`BusSniffer` owns only integration concerns:

- thermal-transient state across measurements
- optional diagnostic entity publishing
- Home Assistant publication throttling
- BLE updates
- decoder lifecycle/orchestration

`measurement_quality.h` was removed because quality is not an independent subsystem; it is part of deciding whether one decoded RT/RH cycle is valid.

No thresholds, score weights, calibration formulas, GPIO assignments, BLE behaviour, or HA publication semantics were intentionally changed.
