<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Historical engineering notes

This directory contains dated notes from reverse engineering, calibration, BLE experiments, power-saving work, and earlier refactors.

They are preserved because they explain *why* some thresholds, protocol choices, and implementation decisions exist. They are **not current API or architecture documentation** and may describe files, state layouts, feature flags, or experiments that have since been superseded.

For the current implementation, use these sources in this order:

1. [`../../README.md`](../../README.md) — current project usage, wiring, build variants, and feature overview.
2. [`../DEVELOPMENT_HISTORY.md`](../DEVELOPMENT_HISTORY.md) — chronological development process and rationale for the current design.
3. [`../ISR_ARCHITECTURE.md`](../ISR_ARCHITECTURE.md) — current interrupt/capture architecture and critical timing rules.
4. The source under [`../../co2_monitor_0601/`](../../co2_monitor_0601/) — authoritative implementation.
5. These historical notes — detailed experiment records and superseded implementations.

Do not copy code or configuration from a historical note without checking it against the current source tree.

- `CALIBRATION_2026-08-17_DUAL_TEMPERATURE.md` — separates RT, physical-air, and Unni LCD temperature views.
- `CALIBRATION_2026-08-17_DISPLAY_HUMIDITY.md` — keeps Unni LCD humidity emulation separate from physical carrier RH.
- `RELIABILITY_HARDENING_2026-08-30.md` — review map, invariants, compatibility contract, and validation debt for the PM/history/BLE/C6 hardening series.
- `HISTORY_DOWNLOAD_CAPTURE_GUARD_2026-08-30.md` — cooperative scheduling of MyAmbience history traffic around passive CO2 capture windows.
- `STANDALONE_SENSIRION_TRH_BRIDGE_2026-08-30.md` — profile-aware shared sample state for the batteryless AHT21/SHT43-compatible bridge and its ENS160 boundary.
