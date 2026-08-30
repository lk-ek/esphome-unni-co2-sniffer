<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# History download versus CO2 capture, 2026-08-30

Status: implemented; host build and hardware timing validation pending.

## Failure

MyAmbience history notifications created enough CPU/controller load to fragment
the passive GPIO capture (`SCL` counts fell from about 130 to 118 and malformed
frames appeared). Capture recovered immediately after the download. Disabling
the sniffer for a 20--30 s transfer would hide the contention but discard normal
six-second CO2 measurements.

## Logical patch series

### [PATCH 1/4] sniffer: expose activity without changing the ISR

`capture_in_progress()` is a task-context snapshot of non-empty/frozen edge
state. It does not use the existing `capturing` flag, which is intentionally
true while the armed sniffer is idle. No logging, BLE call, allocation, or new
bookkeeping enters the ISR.

### [PATCH 2/4] history: predict and guard measurement windows

A valid protocol/CRC result records `last_frame_us`. The period starts at 6 s
and is updated by a 1/8 low-pass step. Elapsed time is divided by the nearest
period multiple before adaptation, so one missed frame does not teach 12 s.
Predictions expire after 30 s without a valid frame.

Notification production pauses at `T-300 ms`, remains paused through
`T+150 ms`, and then resumes from the same packet/sample cursor. It never stops
GATT, disconnects, changes CCCD, or disables GPIO/RMT capture.

### [PATCH 3/4] history: add a reactive backstop

Any observed non-empty I2C capture blocks the next notification immediately.
Normal completion adds a 25 ms tail. A continuously active/stuck capture stops
blocking after 750 ms; prediction and the RMT-SCL protocol-validated recovery
remain independent layers.

### [PATCH 4/4] history: bound transfer lifetime

Download state is reset after 120 s wall clock or 15 s without handing a
notification to the BLE stack. History data and the peer-owned CCCD subscription
are retained. ESPHome's `notify()` surface provides no delivery acknowledgement,
so this is a queue-progress watchdog, not an RF/client-ack watchdog. One bounded
completion/abort log reports guard count and cumulative paused milliseconds.

## Portable contract

`sensirion_history_guard.h` contains no ESP-IDF dependency. Host self-tests cover
prediction boundaries, adaptive/missed-frame normalization, reactive timeout and
tail, stale prediction expiry, and uint32 timestamp wrap in both watchdogs.

## Hardware acceptance

During a full 4096-sample download, record raw SCL/SDA counts, malformed/CRC
errors, RMT repairs, guard pauses, elapsed transfer time, disconnects, and every
accepted CO2 frame. Compare against an otherwise identical unguarded capture.
Accept only if normal CO2 windows remain complete, MyAmbience resumes without
resubscription, total transfer stays below 120 s, and no new RT/RH or power-policy
regression appears.

No build, test, radio measurement, or hardware capture was run for this change.
