<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Refactor V12 — History state consolidation

This refactor simplifies the Sensirion-compatible history implementation without
changing its external protocol or persistent on-flash format.

## Preserved behavior

- 4096-sample logical RAM ring
- 600000 ms default history interval
- 600000 ms flash flush cadence
- `senshist` partition layout: one metadata sector + 14 data sectors
- 7168 physical flash sample slots
- SGH2 metadata format, version 2, 32-byte records, same FNV-1a checksum
- Sensirion service UUID 0x8000 and settings service UUID 0x8100
- characteristics 8001/8002/8003/8004 and their properties
- history download type 7
- 20-byte notifications containing up to two 8-byte T/RH/CO2 samples
- interval writes clear history
- CCCD subscribe starts a download; unsubscribe/disconnect stops it
- persistent restore and append-only flash journal semantics

## Structural changes

The previous implementation had roughly 30 independent global variables with
`sens_history_*` prefixes. They are now grouped by responsibility:

- `HistoryState`: RAM ring and sample cadence
- `FlashState`: partition/journal pointers and flush state
- `GattState`: characteristic/CCCD pointers
- `DownloadState`: one active history transfer

`SensHistorySample` became `std::array<uint8_t, 8>`, allowing direct assignment
from `SensirionSample::encoded()`.

Several one-purpose helpers and repeated long prefixes disappeared. The file is
reduced from 817 to 584 lines while keeping the protocol implementation local
and readable.
