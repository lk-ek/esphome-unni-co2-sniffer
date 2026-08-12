# Historical engineering notes

This directory contains dated notes from reverse engineering, calibration, BLE experiments, power-saving work, and earlier refactors.

They are preserved because they explain *why* some thresholds, protocol choices, and implementation decisions exist. They are **not current API or architecture documentation** and may describe files, state layouts, feature flags, or experiments that have since been superseded.

For the current implementation, use these sources in this order:

1. [`../../README.md`](../../README.md) — current project usage, wiring, build variants, and feature overview.
2. [`../ISR_ARCHITECTURE.md`](../ISR_ARCHITECTURE.md) — current interrupt/capture architecture and critical timing rules.
3. The source under [`../../bus_sniffer/`](../../bus_sniffer/) — authoritative implementation.
4. These historical notes — rationale and development history only.

Do not copy code or configuration from a historical note without checking it against the current source tree.
