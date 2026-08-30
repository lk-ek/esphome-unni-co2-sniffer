<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# ISR architecture and critical decoder code

This document describes the interrupt-driven parts of the Unni sniffer. It is
intended for anyone changing `i2c_sniffer.cpp`, `co2_decoder.cpp`,
`rtrh_decoder.cpp`, pin assignments, timing limits, or debug capture.

The important design rule is simple:

> **The ISRs capture timing and state only. Expensive interpretation happens in
> the normal ESPHome loop.**

Both decoders are passive. They leave the observed GPIOs as inputs without
internal pull-ups or pull-downs and never drive the 0601 signal lines.

## Shared GPIO ISR service

`CO2Monitor0601::initialize_sniffer_io_()` installs the ESP-IDF GPIO ISR service once:

```cpp
gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
```

`i2c_sniffer::setup()` and `rtrh_decoder::setup()` then attach their own handlers
to that shared service. Installing the dispatcher itself IRAM-safe is important:
marking only the leaf handlers `IRAM_ATTR` does not protect the shared GPIO ISR
service from flash-cache-off latency. Code reached from either handler must still
remain short, non-blocking and ISR-safe.

## What must never be added to an ISR

Do **not** put any of the following into either GPIO ISR:

- ESPHome `publish_state()` calls
- BLE/GATT work
- flash writes or preferences
- HTTP/web-server operations
- logging (`ESP_LOG*`)
- dynamic allocation (`new`, `malloc`, `std::vector`, `std::string`, ...)
- mutex/semaphore waits
- delays
- floating-point calibration or statistics that can be deferred

The handlers can fire very frequently during an active sensor cycle. Their job
is to copy the minimum information needed to reconstruct the signal later.

---

# I²C / CO₂-bus ISR

Source: `co2_monitor_0601/i2c_sniffer.cpp`

## Pins

On the XIAO ESP32-C3 configuration used by this project:

- SDA: GPIO6 / D4
- SCL: GPIO7 / D5

Both pins use `GPIO_INTR_ANYEDGE` and both call the same `gpio_isr()`.

## What happens on every edge

The passive I²C ISR does this:

1. Return immediately if capture is paused.
2. Snapshot **both** bus pins as early as possible and encode them into a two-bit state.
   On ESP32-C3/C6 this is one atomic read of `GPIO_IN_REG`; other ESP32 families
   use the portable `gpio_get_level()` fallback.
3. Ignore the interrupt immediately if the combined state is unchanged.
4. Read the current microsecond timestamp using `esp_timer_get_time()` only for a real state transition.
5. Remember the bus state that existed before the first captured transition.
6. Store `{timestamp, combined_state}` in the fixed capture buffer.
7. If the buffer is full, mark the capture as overflowed and finished.

The single-register C3/C6 snapshot both shortens the hottest part of the ISR and
prevents a transition between two separate SCL/SDA reads from creating a bus
state that never physically existed. The timestamp deliberately remains based
on `esp_timer_get_time()`: replacing it with the CPU cycle counter would make
recovery thresholds and debug-export timing depend on CPU-frequency/light-sleep
assumptions for relatively little guaranteed benefit.

The buffer contains 4096 entries, stored as two aligned arrays:

```cpp
struct EdgeBuffer {
  volatile uint32_t t[4096];
  volatile uint8_t value[4096];
};
```

Keeping timestamp and state in separate arrays is deliberate. A conventional
`{uint32_t,uint8_t}` array is padded to 8 bytes per entry on ESP32, consuming
32 KiB. The structure-of-arrays representation preserves the same 4096-edge
capture depth and LA02 data while using exactly 20 KiB.

The ISR also reads the combined SDA/SCL state before taking the timestamp or
updating diagnostic counters. This minimizes the latency between interrupt entry
and observing the bus, reducing the chance that a second edge is already present
when both pins are sampled. No I²C decoding occurs in the ISR.

## Optional RMT-SCL hardware assist

On ESP32-C3/C6 the component can enable an additional RMT RX channel on SCL:

```yaml
co2_monitor_0601:
  i2c_capture_backend: rmt_scl
```

The shipped `i2c-sniffer-debug.yaml` enables this backend. Normal GPIO capture
remains active on both SDA and SCL and remains the absolute-time/raw-debug source.
RMT is deliberately used only as an SCL hardware oracle.

A seemingly simpler design with one RX-RMT channel for SDA and one for SCL was
rejected after checking the ESP-IDF driver semantics: RX transactions begin on
the first level change of each individual input, and the RMT sync manager is TX
only. Two RX channels therefore have no guaranteed common epoch. Merging them as
if their relative timestamps shared time zero could create false START/STOP or
bit timing.

The implemented path instead uses:

- 5 MHz RMT resolution (0.2 us/tick),
- the native C3/C6 48-symbol hardware block,
- RX ping-pong/partial receive into a fixed 128-symbol SRAM accumulator,
- RMT interrupt priority 3,
- `CONFIG_RMT_RX_ISR_CACHE_SAFE`,
- 5 ms signal-idle termination, matching the GPIO capture quiet period.

No RMT data is used when the GPIO capture already passes the caller's strict
protocol validator. When GPIO decoding fails, the decoder fits the relative RMT
SCL edge train against the SCL edges that the GPIO stream did observe. Only a
high-confidence alignment is accepted. Hardware-observed SCL transitions that
are missing from GPIO are then inserted **temporarily**, preserving SDA from the
GPIO state at that instant, and the complete capture is decoded again.

The candidate is accepted only if the same strict CO2 validator succeeds (frame
shape, address/direction, ACK/NACK pattern and Sensirion CRC). Otherwise every
inserted edge is rolled back and the existing bounded software missing-clock
recovery gets its normal chance. Even on success the insertions are rolled back
after decoding: `/capture`, LA02, UDP debug and edge diagnostics continue to
represent the raw GPIO observation, not a repaired waveform.

This architecture is specifically intended for the observed failure under heavy
BLE/GATT history traffic, where GPIO ISR latency can span a complete short SCL
pulse. RMT records SCL durations in peripheral memory even while the CPU is busy;
the callback only copies fixed-size symbol chunks into SRAM.

## Cooperative BLE-history capture guard

The history sender is the controllable source of that BLE load. It therefore
stops queueing notifications 800 ms before an adaptively predicted CO2 or RT/RH
completion until at least 150 ms afterwards. The wider lead time lets already
queued BLE host/controller work drain. Protocol/CRC-valid CO2 frames and
completed RT/RH snapshots update independent roughly 6 s/30 s estimators. The
RT/RH completion anchor is shifted back 500 ms to approximate the cycle start
(about 383 ms waveform plus 100 ms silence); missed cycles are normalized to a
multiple of the current estimate. CO2/RT/RH predictions expire after 30/90
seconds without a new anchor.

Prediction is backed by a reactive task-context snapshot. The ISR still only
records edges; the I2C and RT/RH `capture_in_progress()` probes report whether
either decoder has live work. History transmission pauses immediately when the
loop observes that state, retains a 25 ms tail after normal completion, and
ignores a continuously stuck state after 750 ms. RMT-SCL assist remains enabled
and neither ISR is disabled by the history scheduler.

Ordinary pauses preserve the BLE connection, CCCD subscription, packet sequence,
and sample cursor. The total/no-progress watchdogs abort only transfer state and
never erase history. ESPHome's `notify()` API has no delivery acknowledgement;
for this watchdog, progress means a notification was handed to the BLE stack.

The decoded `Capture` also carries its raw pre-repair SCL-transition count,
calculated in task context from the frozen edge buffer. During an active history
download, fewer than 130 raw SCL transitions or a frame error raises the next
predictive lead by 250 ms (maximum 500 ms extra). Three clean captures reduce
the extra lead by 50 ms. This feedback adds no ISR work and does not replace the
always-enabled RMT-SCL safety net; isolated RMT repairs also occur without a
history download.

## Protocol-validated GPIO missing-clock recovery

The GPIO path can miss an entire SCL pulse when ISR latency spans both edges.
For rejected captures, the generic sniffer examines **constant-SCL level
intervals**, not just gaps between adjacent samples. This matters because SDA
may still change while a complete SCL high/low pulse is missed. Unusually long
HIGH or LOW intervals generate bounded hypotheses for one missing opposite-level
pulse, and if no one-pulse hypothesis succeeds the decoder may test two pulses
in different intervals.

The generic layer never treats timing as proof. The CO₂ integration accepts a
reconstruction only when it yields one complete `0x62 W EC 05` frame followed
by one `0x62 R` measurement frame with the exact ACK/NACK pattern and valid
Sensirion CRC. Multiple timing placements are acceptable only when they decode
to the **same** bus transaction; competing valid protocol results cause the
recovery to be rejected. At most two missing clocks are reconstructed. The raw
GPIO capture is restored bit-for-bit after every candidate and remains unchanged
for `/capture`.

## Why both pins use the same handler

An I²C START/STOP condition depends on the relationship between SDA and SCL, not
on one pin in isolation. The ISR therefore records the complete bus state after
any SCL or SDA transition.

If nearly simultaneous changes produce more than one IRQ, this check filters
redundant work:

```cpp
if (value == last_value) return;
```

The opposite failure mode is also possible: a delayed ISR can observe an SDA
change and an SCL rise only after both have happened. When that combined sample
occurs in the middle of a byte, the framer treats it as SDA setup followed by
the SCL sampling edge, rather than misclassifying it as START/STOP. This rule
was derived from a captured real transaction and is intentionally limited to
positions where START/STOP would not be a legal byte boundary.

## Why `capture_initial_value` matters

The first stored sample is the state **after** an edge. START/STOP recognition
also needs to know the state before that first edge. The ISR therefore preserves
`last_value` as `capture_initial_value` when `sample_count == 0`.

Removing that field can make the first transaction in a capture undecodable.

## Handoff to the normal loop

`i2c_sniffer::poll()` waits until the bus has been quiet for at least 5000 µs:

```cpp
CAPTURE_TIMEOUT_US = 5000;
```

It then sets `capturing = false`, decodes the frozen edge buffer in normal task
context, resets the capture state, and enables capture again.

The generic I²C layer reconstructs START, repeated START, STOP, rising-clock
data bits, 7-bit address/direction and ACK/NACK bits into fixed-size
`i2c_sniffer::Frame` objects. Frames use 7-bit addressing and retain up to 32
data bytes in fixed storage. Each frame also carries a structural `FrameStatus`;
incomplete bytes, captures ending inside a frame and overlong/truncated frames
are classified as malformed and counted once at the generic I²C layer. They are
not passed to `co2_decoder::process_frame()`. Only structurally valid frames can
reach the CO₂ protocol layer, which alone knows about address 0x62, command
0xEC05, the Sensirion-compatible CRC or ppm values.

The SCL rise immediately before STOP or repeated START is initially
indistinguishable from the first data bit of a following byte. The decoder
therefore rolls back that single speculative bit when the subsequent SDA edge
confirms the bus boundary. Do not remove this handling when tightening frame
validity checks.

In debug-capture builds, malformed frames, valid frames not claimed by
`co2_decoder`, and structurally valid CO₂ frames rejected by CRC/ACK/length
checks are logged separately from normal task context. The first suspicious
transaction freezes the corresponding raw edge buffer; later
captures cannot overwrite it until `/capture` is downloaded successfully. The
HTTP handler streams the binary trace in moderate chunks and only releases the
freeze after the terminating chunk succeeds; a failed transfer therefore leaves
the same trace available for retry. Raw captures use `LA02`, which includes the
bus state before the first edge so the first START/STOP transition can be
reconstructed exactly. None of this processing
runs in the ISR.

### Critical point

Do not move I²C framing or CO₂ CRC processing into `gpio_isr()`. The current
split is intentional: edge capture is timing-sensitive; frame/protocol decoding
is not.

### Concurrency note

The I²C capture state is shared through `volatile` variables. `volatile` forces
actual memory accesses; it is **not** a mutex and does not make multi-step state
changes atomic. The current design minimizes the overlap by stopping capture
before decoding and only restarting after the buffer has been reset.

If this code is ever changed to use another task/core or to decode concurrently,
introduce an explicit ISR-safe synchronization strategy rather than assuming
`volatile` provides synchronization.

---

# RT/RH ISR

Source: `co2_monitor_0601/rtrh_decoder.cpp`

## Pins

The RT/RH decoder observes two original Unni signals:

- RT: GPIO3 / D1
- RH: GPIO4 / D2

Both pins use `GPIO_INTR_ANYEDGE`. XIAO D3/GPIO5 is reserved for USB/VBUS detection and is not part of the RT/RH ISR path.

The ISR receives an encoded pin index through its `void *arg`; this lets one
handler distinguish which physical pin caused the interrupt without registering
two separate functions.

## Decoder state

All time-critical RT/RH state is grouped in `DecoderState`:

```cpp
DecoderState decoder;
```

Important members are:

- `collecting`: currently inside a sensor measurement sequence
- `measurement_start_us`: timestamp used to determine REF/RT/RH phase
- `last_edge_us`: most recent observed edge, used for end-of-measurement detection
- `gpio_state`: last combined RT/RH state
- `last_rt_fall_us`: previous RT falling edge
- `have_rt_rise`: protects against accepting an incomplete RT period
- `phase`: REF, RT or RH
- `ref`, `rt`, `rh`: accumulated RT period sums/counts
- `rt_temperature_*`: first 880 RT periods used for temperature
- `rh_state`: legacy RH-state recurrence samples retained for diagnostics; production RH uses the RH carrier period
- `snapshot`: completed measurement handed to normal task context

## Phase selection is time-based

One of the most important implementation details is that the phase is **not**
inferred from measured RC periods or edge counts.

Instead, elapsed time since the first edge selects the phase:

- REF: `< 125000 µs`
- RT: `125000 .. <252000 µs`
- RH: `>= 252000 µs`

This is deliberate. The measured RC periods are the values we want to measure;
using them to identify the phase would make the classification depend on the
measurement itself.

When a phase boundary is crossed, the decoder clears `last_rt_fall_us` and
`have_rt_rise` so a period can never straddle two phases.

**Changing `REF_PHASE_END_US` or `RT_PHASE_END_US` changes the meaning of every
subsequent period accumulation. Treat these constants as critical calibration /
protocol constants.**

## Only the physical RT IRQ measures RT periods

The ISR reads the complete two-pin RT/RH state, but REF/RT/RH period timing is
updated only if the interrupt was caused by physical RT:

```cpp
const bool is_rt_irq = pin_index == 0;
```

This is critical when multiple lines change almost simultaneously. If a RH interrupt were allowed to infer a RT transition from the aggregate state,
IRQ ordering could create a false period.

A valid RT period requires:

1. a RT rising edge (`have_rt_rise = true`), then
2. the next RT falling edge, and
3. a period no longer than `CYCLE_MAX_US` (20000 µs).

The resulting falling-edge-to-falling-edge period is accumulated into the
currently selected phase.

## Temperature period selection

During RT, every valid RT period contributes to the RT phase statistics, but
only the first 880 periods contribute to the temperature calculation:

```cpp
RT_TEMP_CYCLES = 880;
```

This distinction is visible in `Measurement` as `rt_phase_count` versus
`rt_count`.

Do not casually merge these two counters; one describes the whole RT phase,
while the other describes the subset used for temperature.

## RH-state timing

During RH the decoder also watches for the characteristic combined state:

```text
RT=0, RH=1
```

In bit form the test is:

```cpp
(state & 0x09) == 0x08
```

Only the RT and RH signals are required by the production decoder.

The time between recurrences of this state is stored in a 96-entry ring:

```cpp
RH_STATE_PERIOD_SAMPLES = 96;
```

Only intervals from 40 to 60000 µs are accepted. Outside the ISR, the decoder
sorts the captured values and uses their median. The median calculation is
intentionally outside the handler.

## Why the ISR does not finalize a measurement

The RT/RH sequence does not end with a dedicated digital end marker. The normal
`rtrh_decoder::loop()` considers the measurement complete after 100 ms with no
new edge:

```cpp
MEASUREMENT_QUIET_US = 100000;
```

The accepted RH-state interval can be as long as 60 ms, so 100 ms remains above
the longest valid intra-measurement gap while avoiding the historical 15-second
post-measurement delay. On the tested hardware the full REF+RT+RH waveform lasts
about 383 ms, so a normal snapshot is ready roughly half a second after the first
edge.

Before copying the live accumulator into `decoder.snapshot`, it disables both RT/RH GPIO interrupts, checks the quiet condition again, performs the
copy, refreshes current pin levels, and re-enables the interrupts.

That second check is important. An edge may occur between the first quiet-time
check and interrupt disable. Without the re-check, the code could finalize a
measurement that was no longer actually quiet.

This short interrupt-disabled section is the RT/RH decoder's critical handoff
between ISR-owned live state and task-owned snapshot processing.

Do not perform logging, calibration, BLE work, HA publishing, HTTP work, or any
other slow operation while those GPIO interrupts are disabled.

## Snapshot -> derived measurement

After finalization, `poll()` operates on the completed snapshot in normal task
context. It performs:

1. mean period and duration calculations
2. RH-state median
3. RT/REF and RH/REF ratios
4. validity/reject checks
5. measurement quality calculation
6. temperature calibration
7. temperature-compensated humidity calibration
8. calibration extrapolation flags

`CO2Monitor0601` then adds the thermal-transient state and publishes the result to
ESPHome/BLE.

That boundary should remain intact:

```text
GPIO ISR
  -> integer edge/timing accumulation
  -> quiet-time snapshot
  -> floating-point derive/calibration
  -> ESPHome + BLE publishing
```

---

# RT/RH debug capture

When `debug_capture: true`, the normal RT/RH ISR also maintains a separate
`DebugCaptureState`.

This is deliberately secondary to the production decoder. The normal decoder
updates happen first; only afterwards does the debug block decide whether an
edge should be retained.

The debug capture is heavily decimated:

- regular state changes: every 16th edge
- unusual states: every 8th unusual edge
- maximum stored samples: 1536
- capture window: 450000 µs

The capture therefore does **not** represent every raw edge. It is a diagnostic
view intended to keep memory/ISR cost bounded.

When `debug.ready` is true, the ISR stops collecting debug samples but continues
normal RT/RH decoding.

HTTP serialization for `/rt_rh_capture.csv` and `/rt_rh_timing.csv` occurs in
web-server/task context, never in the ISR. The raw capture CSV is emitted in
roughly 1 KiB chunks; if the client disconnects or a socket send times out, the
ready debug capture is retained so the same snapshot can be requested again.

---

# Safe modification checklist

Before changing ISR-related code, check all of the following:

1. Does the ISR still perform only fixed-cost, non-blocking work?
2. Did any logging, allocation, BLE, web, flash or ESPHome publishing enter the
   ISR path?
3. Are phase boundaries still elapsed-time based?
4. Are RT periods still measured only from a physical RT IRQ?
5. Can a RT period accidentally cross a REF/RT/RH phase boundary?
6. Is the first CO₂ bus state before the first captured edge still preserved?
7. Are I²C framing and CO₂ decoding still done only after capture is stopped?
8. Is RT/RH snapshot copying still protected by disabling/rechecking the two
   RT/RH GPIO interrupts?
9. Are ISR-shared fields still fixed-size and allocation-free?
10. Have both `debug_capture: false` and `debug_capture: true` builds been tested?
11. After flashing, do CO₂, RT/RH quality and MyAmbience history still behave
    normally?

If a change affects timing constants, pin assignments, state masks, sample
limits, buffer sizes, or ISR synchronization, treat it as a protocol-level
change rather than a cosmetic refactor and validate it against raw captures.

## Automatic Light-sleep interaction

When `light_sleep: true`, the first RT/RH GPIO ISR edge calls
`power_save::on_rtrh_edge_from_isr()`. That function acquires an
`ESP_PM_NO_LIGHT_SLEEP` lock. `esp_pm_lock_acquire()` is used here specifically
because ESP-IDF permits it from ISR context. The lock is guarded so subsequent
RT/RH edges do not recursively acquire it.

Ownership is tracked independently for the NO_LIGHT_SLEEP and CPU-frequency
locks. A partial acquire is rolled back when possible; a failed rollback or
release retains the corresponding owned state so task context retries only the
still-held handle. USB and CO2-window lock pairs use the same per-handle rule,
preventing both recursive underflow and a falsely unlocked state after one half
of a pair fails.

Do not move the first-edge lock acquisition out of the ISR: automatic
Light-sleep between RT/RH edges would add wake latency to ISR timestamps and
could corrupt period measurements. The lock is released only from normal task
context after RT/RH completion plus a subsequent valid CO2 frame, or the
failsafe timeout.

### Deferred edge diagnostics (2026-08-30)

The GPIO ISR keeps only the exact ISR-entry counter. State/SCL/SDA transition
counts are reconstructed from the captured edge stream in the main loop and the
currently active buffer is included when the 5-second diagnostic is printed.
This preserves the diagnostic semantics while removing three volatile
read/modify/write operations from every captured edge.

The ISR also checks the atomic GPIO snapshot against `last_value` before calling
`esp_timer_get_time()`. Spurious/coalesced GPIO interrupts which no longer
represent a new bus state therefore return without paying for a 64-bit timer
read. Real edge timestamps remain on the same ESP Timer microsecond timebase;
no CPU-cycle conversion or frequency/sleep assumption is introduced.
