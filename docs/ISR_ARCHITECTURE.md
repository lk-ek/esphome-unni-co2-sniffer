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
internal pull-ups or pull-downs and never drive the 0601 signal lines. The
ESP-IDF RMT RX allocator currently enables a pull-up as an implementation side
effect; `i2c_sniffer` immediately disables that pull-up again after allocating
the SCL RMT channel.

## Shared GPIO ISR service

`CO2Monitor0601::initialize_sniffer_io_()` installs the ESP-IDF GPIO ISR service once:

```cpp
gpio_install_isr_service(0);
```

`i2c_sniffer::setup()` and `rtrh_decoder::setup()` then attach their own handlers
to that shared service.

The service is installed with flags `0`, not `ESP_INTR_FLAG_IRAM`. The handlers
are nevertheless marked `IRAM_ATTR`, as are the small RT/RH helper functions that
are called directly from its ISR path. Do not interpret `IRAM_ATTR` alone as a
promise that arbitrary code may safely be called from the handler: code reached
from an ISR still needs to be short, non-blocking and ISR-safe.

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
2. Read the current microsecond timestamp using `esp_timer_get_time()`.
3. Read **both** bus pins and encode them into a two-bit state.
4. Ignore the interrupt if the combined state is unchanged.
5. Remember the bus state that existed before the first captured transition.
6. Store `{timestamp, combined_state}` in the fixed capture buffer.
7. If the buffer is full, mark the capture as overflowed and finished.

The buffer contains 4096 entries:

```cpp
struct Sample {
  uint32_t t;
  uint8_t value;
};
```

No I²C decoding occurs in the ISR.

## RMT SCL hardware assist

RMT is experimental and disabled by default. The production path remains the
IRAM-safe shared GPIO ISR plus task-context decoding.

When explicitly enabled, an ESP32-C3 RMT RX channel independently timestamps
SCL pulse durations. `rmt_receive()` is pre-armed from normal task context while
the bus is idle; ESP-IDF starts actual reception at the first input level
change. The first GPIO edge records the matching absolute start timestamp for
GPIO/RMT alignment. No RMT driver API is called from the GPIO ISR.

The channel reserves 96 hardware symbols (two 48-symbol blocks) and uses
interrupt priority 1. A normal EC05 command plus response therefore fits in the
hardware buffer without the 48-symbol ping-pong copy path used by the first RMT
experiment. Cache-safe RMT ISR and `rmt_receive()`-in-IRAM Kconfig options are
not forced; with the whole short transaction fitting in hardware, completion
can be deferred rather than competing with Wi-Fi/BLE during cache-off windows.

RMT pulse boundaries are converted to absolute timestamps. After an early
matching GPIO/RMT edge pair is found, RMT must remain a strict superset/match of
the GPIO SCL timeline. Only extra RMT edges are inserted into the working copy.
If alignment is ambiguous, RMT is truncated/full, or RX does not finish within
a bounded wait, no RMT repair is attempted.

The original GPIO waveform is preserved unchanged for `/capture`, including
when RMT repairs the working decoder copy. The RMT channel is disabled whenever
the battery power policy disables CO₂ capture.

## Protocol-validated GPIO missing-clock recovery

The GPIO path can miss an entire SCL pulse when ISR latency spans both edges.
For rejected captures, the generic sniffer may insert one synthetic full SCL
pulse into an unusually long event gap and re-run framing. The generic layer
does not decide that the result is correct: a caller-supplied validator must
accept exactly one candidate. The CO₂ integration requires one complete
`0x62 W EC 05` frame followed by one `0x62 R` measurement frame with the exact
ACK/NACK pattern and valid Sensirion CRC. Zero or multiple valid candidates mean
no repair. The raw capture is never modified for download.

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
- `rh_state`: RH-state recurrence samples used for the RH median
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

Do not move the first-edge lock acquisition out of the ISR: automatic
Light-sleep between RT/RH edges would add wake latency to ISR timestamps and
could corrupt period measurements. The lock is released only from normal task
context after RT/RH completion plus a subsequent valid CO2 frame, or the
failsafe timeout.
