<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Sparse history time anchors, 2026-08-30

Status: implemented and extended on 2026-08-31 with a sparse completed-run index and rolling multi-download session; ESP32/MyAmbience multi-run hardware validation pending.

## Why

The Sensirion Gadget history wire format describes one uniformly sampled run: one
interval, one age for the newest sample, then sample packets. The official
implementation does not provide an in-band gap/segment marker. Sending multiple
headers in one subscription would therefore be an undocumented protocol extension.

The previous local persistence retained samples across reboot but reset their
runtime age, which could make old data appear newer than it was.

## Persistence model

Metadata version 4 keeps the existing 32-byte record size, A/B journal sectors,
14 data sectors, and 8-byte sample representation. The two words that were
reserved in V3 now hold only the newest continuous run's sparse timing state:

- `reserved0`: Unix UTC seconds of the first sample in the newest run, or zero
  while no wall clock is available;
- `reserved1`: number of samples in that run.

V4 protects both fields with its metadata checksum. V2 and V3 records keep their
legacy checksum rule and remain readable. Because they contain no trustworthy run
time state, migration preserves their samples but exposes no old run to
MyAmbience until a new run begins.

No timestamp is added to each sample. A new run starts only after reboot, after a
full sampling interval was missed, or after a material wall-clock correction. If
wall-clock synchronization arrives after a run has already started, the first
sample's UTC anchor is reconstructed once from the known cadence.

## Time source

The API-enabled shipped variants use ESPHome's Home Assistant time source. The
BLE-only build deliberately has no network/API time source. It can expose the
current boot's run using relative age, but after reboot it does not invent an age
for restored samples until an absolute clock is available.

## MyAmbience compatibility

The GATT UUIDs, type-7 header, 20-byte notifications, sample bytes, and packet
sequence remain unchanged. Runs are never concatenated across a gap inside one
Sensirion transfer.

The bridge now treats consecutive downloads as a rolling five-minute session.
The first completed transfer sends the newest continuous run. Each later
completed transfer within five minutes sends the next older retained run and
resets the timeout. Disconnects preserve this cursor; aborted transfers do not
advance it. After five minutes without a completed transfer, the next request
starts at the newest run again. The count characteristic tracks the run selected
by this cursor.

To make older runs reconstructable without per-sample timestamps, each run is
finalized only when a gap starts the next run. At that point one 8-byte sparse
record is persisted in ESPHome NVS containing the completed run's UTC anchor,
physical flash start slot, and sample count. Up to 128 completed runs are kept.
Entries are used only while their complete sample range is still inside the
4096-sample flash window and their age is representable by the Sensirion header.
Runs without a trustworthy wall-clock anchor are skipped rather than assigned a
false age. Existing V4 samples are unchanged and remain readable; older gaps
that occurred before this run index existed cannot be inferred retroactively.

This deliberately favors correct timestamps over pretending that separated runs
were sampled continuously.
