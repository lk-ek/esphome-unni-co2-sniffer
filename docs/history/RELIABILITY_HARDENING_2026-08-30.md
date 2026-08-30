<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Reliability hardening, 2026-08-30

Status: implemented; host, target, radio, power, and PCB validation pending.

This note is the review map for one large working-tree change. It records the
logical patch order, compatibility contract, and remaining proof obligations.
Calibration, protocol bytes, passive-tap safety, and the history sample/data
layout are intentionally unchanged.

## Virtual patch series

### [PATCH 1/8] power: track each PM lock independently

Problem: one Boolean represented two PM locks. A partial acquire/release could
therefore cause an underflow or permanently prohibit Light Sleep.

Change: `PmLockPair` records sleep/CPU ownership separately, rolls back partial
acquires, preserves unreleased ownership, counts failures, and retries bounded
task-context reconciliation. RT/RH ISR ownership is likewise split; task-side
release remains interrupt-masked.

Invariant: software may clear ownership only after that exact lock was released.

### [PATCH 2/8] capture: make setup failure atomic

Problem: partial GPIO/ISR setup was retried over installed handlers and
`rtrh_decode_only` was omitted from setup and power paths.

Change: I2C and RT/RH modules gained teardown paths; failed setup unwinds owned
resources and permanent failure calls `mark_failed()`. Decode-only participates
in GPIO, debug, and PM setup. Schema validation makes the three RT/RH capture
modes exclusive and requires an active mode for `rtrh_gpio_setup`.

Invariant: setup either owns a complete capture path or no capture resources.

### [PATCH 3/8] history: journal metadata and retry checkpoints

Problem: a successful data write followed by failed metadata write lost the
retry state and left one metadata copy as a single point of failure.

Change: metadata version 3 uses A/B journals in 4-KiB erase sectors 0 and 15 of
the 64-KiB partition. Data sectors 1..14, sample encoding, capacity, and GATT
wire format stay unchanged.
Version 2 is accepted and migrated on the next flush. Generation comparison is
serial-number/wrap safe. `metadata_dirty` remains set until a checkpoint succeeds.

Invariant: data becoming durable never implies that its index checkpoint is
durable. At least one previously valid journal remains recoverable.

### [PATCH 4/8] history: move destructive work out of GATT callbacks

Problem: an arbitrary interval (for example 1 ms) could force continuous flash
traffic, and callbacks erased flash or synchronized preferences inline.

Change: history intervals are accepted only from 60 s through 24 h. An interval
change queues `CLEARING`; sampling/download pause and the loop erases at most one
sector per iteration before committing a fresh journal. Device settings are
copied in the callback and persisted, with retry, in loop context; OTA requests a
final flush. Download now reuses its 20-byte value buffer, requires connection
and CCCD state, and aborts on disconnect or flash-read failure.

ESPHome currently sends the successful ATT write response before invoking this
external-component callback. Invalid values are rejected and restored locally,
but this layer cannot replace that response with an ATT error.

Invariant: BLE callbacks do no erase/write/sync work; destructive progress is
bounded and visible to the main state machine.

### [PATCH 5/8] runtime: compile production diagnostics out

Problem: temporary stage timers and slow housekeeping ran in the fast component
loop in every build.

Change: `runtime_diagnostics` defaults to false and gates stage timing at compile
time; the debug YAML enables it. Stable VBUS polling is 200 ms (10 ms while
debouncing), and idle history sampling is deadline-gated while active capture and
download remain responsive. USB/Battery BLE defaults are 2 s/3 s. The SHT43 RAM
probe disables unrelated raw capture.

Invariant: no deadline may gate I2C completion, RT/RH silence finalization, an
open capture window, or an active BLE transfer.

### [PATCH 6/8] BLE/config: make identity changes explicit and validate inputs

Change: `ble_identity_mode` is `legacy_fixed` by default or opt-in
`device_derived`. The latter derives a stable locally administered address and
unique device ID from eFuse identity; selecting it can invalidate bonds and app
caches. BLE names are checked as UTF-8 bytes (1..31, no NUL). Time values receive
semantic limits; active probing requires the sniffer; debug UDP requires debug
capture. Sample serialization rejects non-finite inputs and bounds encoded
temperature/humidity.

Invariant: existing installations retain the legacy BLE identity unless their
configuration explicitly opts out.

### [PATCH 7/8] tests/CI/repository: restore executable contracts

Change: host tests cover history bounds/generation wrap and BLE golden payloads;
CI calls `python3 tests/host/run_host_tests.py` instead of the absent
`tests/test.sh`. `.gitignore` covers secrets, Finder files, local ZIP snapshots,
and `unni-debug/`; no local artifact is deleted.

### [PATCH 8/8] C6: turn DRC drift into a failure

Change: the validator uses explicit warning-category ceilings and treats
`via_dangling` as fatal. The redundant dangling `CONN_BTN` via at
`(147.2, 135.53)` was removed without changing the connected B.Cu route.

Invariant: KiCad 10.99 sources remain canonical; generated 10.0.5 files are not
edited as a second source of truth.

## Compatibility contract

| Surface | Result |
|---|---|
| Existing YAML | Defaults preserve behavior except corrected USB BLE interval (2 s) |
| New YAML | `runtime_diagnostics`; `ble_identity_mode` |
| Flash | V2 readable; next flush migrates metadata to redundant V3 |
| History data/GATT | Sample bytes, capacity, data sectors, and download format unchanged |
| BLE identity | Fixed by default; device-derived mode is an explicit migration |
| Measurement | Calibration, plausibility envelopes, protocol decoding unchanged |

## Review and validation order

Review patches 1--4 first: they change failure semantics and persistence. Then
review schema/code-generation pairs, followed by tests, documentation, and C6.

Run, in order:

```sh
python3 tests/host/run_host_tests.py --mode config
python3 tests/host/run_host_tests.py --mode compile
python3 tests/host/run_host_tests.py
```

Then exercise all five shipped YAML variants on ESPHome 2026.7.4. Hardware must
cover USB/battery transitions, Energy Save, RT/RH and CO2 wake windows, OTA with
dirty state, MyAmbience pairing, invalid history writes, disconnect during
download, and a complete 4096-sample transfer. Compare capture rejects, API/BLE
stability, awake time, Light-Sleep residency, and energy under identical radio
conditions.

For C6, regenerate the DRC report from canonical sources before running
`unni-smartification-c6/validate_all.sh`; the checked report predates the via
edit. No PCB success or hardware behavior is inferred from static checks.

## Deliberately deferred

- Splitting the orchestrator into battery/connectivity/publication components.
- Fake-partition power-cut and PM-lock fault-injection suites.
- Silkscreen/footprint cleanup, pinned KiCad CI, and artifact/LFS migration.
- Any claim of timing, radio, sleep-residency, energy, or PCB validation.

No build or test was run while preparing this change, by handoff agreement.
