# ISR architecture and critical decoder code

This document describes the interrupt-driven parts of the Unni sniffer. It is
intended for anyone changing `co2_decoder.cpp`, `rtrh_decoder.cpp`, pin
assignments, timing limits, or debug capture.

The important design rule is simple:

> **The ISRs capture timing and state only. Expensive interpretation happens in
> the normal ESPHome loop.**

Both decoders are passive. They configure their GPIOs as inputs without pull-ups
or pull-downs and never drive the Unni signal lines.

## Shared GPIO ISR service

`BusSniffer::initialize_sniffer_io_()` installs the ESP-IDF GPIO ISR service once:

```cpp
gpio_install_isr_service(0);
```

`co2_decoder::setup()` and `rtrh_decoder::setup()` then attach their own handlers
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

# CO₂ ISR

Source: `bus_sniffer/co2_decoder.cpp`

## Pins

On the XIAO ESP32-C3 configuration used by this project:

- SDA: GPIO6 / D4
- SCL: GPIO7 / D5

Both pins use `GPIO_INTR_ANYEDGE` and both call the same `gpio_isr()`.

## What happens on every edge

The CO₂ ISR does this:

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

## Why both pins use the same handler

An I²C START/STOP condition depends on the relationship between SDA and SCL, not
on one pin in isolation. The ISR therefore records the complete bus state after
any SCL or SDA transition.

If nearly simultaneous changes produce more than one IRQ, this check filters
redundant work:

```cpp
if (value == last_value) return;
```

## Why `capture_initial_value` matters

The first stored sample is the state **after** an edge. START/STOP recognition
also needs to know the state before that first edge. The ISR therefore preserves
`last_value` as `capture_initial_value` when `sample_count == 0`.

Removing that field can make the first transaction in a capture undecodable.

## Handoff to the normal loop

`co2_decoder::poll()` waits until the bus has been quiet for at least 5000 µs:

```cpp
CAPTURE_TIMEOUT_US = 5000;
```

It then sets `capturing = false`, decodes the frozen edge buffer in normal task
context, resets the capture state, and enables capture again.

The decoder reconstructs START, STOP, rising-clock data bits and ACK/NACK bits.
Only then does it validate the Sensirion CRC and expose a CO₂ value.

### Critical point

Do not move `decode_capture()` or CRC processing into `gpio_isr()`. The current
split is intentional: edge capture is timing-sensitive; frame decoding is not.

### Concurrency note

The CO₂ capture state is shared through `volatile` variables. `volatile` forces
actual memory accesses; it is **not** a mutex and does not make multi-step state
changes atomic. The current design minimizes the overlap by stopping capture
before decoding and only restarting after the buffer has been reset.

If this code is ever changed to use another task/core or to decode concurrently,
introduce an explicit ISR-safe synchronization strategy rather than assuming
`volatile` provides synchronization.

---

# RT/RH ISR

Source: `bus_sniffer/rtrh_decoder.cpp`

## Pins

The RT/RH decoder observes two original Unni signals:

- G10: GPIO3 / D1
- G13: GPIO4 / D2

Both pins use `GPIO_INTR_ANYEDGE`. D3/GPIO5 (G11) was used during reverse engineering but is not required by the production decoder.

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
- `gpio_state`: last combined G10/G13 state
- `last_g10_fall_us`: previous G10 falling edge
- `have_g10_rise`: protects against accepting an incomplete G10 period
- `phase`: REF, RT or RH
- `ref`, `rt`, `rh`: accumulated G10 period sums/counts
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

When a phase boundary is crossed, the decoder clears `last_g10_fall_us` and
`have_g10_rise` so a period can never straddle two phases.

**Changing `REF_PHASE_END_US` or `RT_PHASE_END_US` changes the meaning of every
subsequent period accumulation. Treat these constants as critical calibration /
protocol constants.**

## Only the physical G10 IRQ measures G10 periods

The ISR reads the complete two-pin G10/G13 state, but REF/RT/RH period timing is
updated only if the interrupt was caused by physical G10:

```cpp
const bool is_g10_irq = pin_index == 0;
```

This is critical when multiple lines change almost simultaneously. If a G13 interrupt were allowed to infer a G10 transition from the aggregate state,
IRQ ordering could create a false period.

A valid G10 period requires:

1. a G10 rising edge (`have_g10_rise = true`), then
2. the next G10 falling edge, and
3. a period no longer than `CYCLE_MAX_US` (20000 µs).

The resulting falling-edge-to-falling-edge period is accumulated into the
currently selected phase.

## Temperature period selection

During RT, every valid G10 period contributes to the RT phase statistics, but
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
G10=0, G13=1
```

In bit form the test is:

```cpp
(state & 0x09) == 0x08
```

D3/G11 was removed after a live shadow-decoder comparison showed identical RH median, sample count, and event count for the two-pin and three-pin state definitions across the validation captures.

The time between recurrences of this state is stored in a 96-entry ring:

```cpp
RH_STATE_PERIOD_SAMPLES = 96;
```

Only intervals from 40 to 60000 µs are accepted. Outside the ISR, the decoder
sorts the captured values and uses their median. The median calculation is
intentionally outside the handler.

## Why the ISR does not finalize a measurement

The RT/RH sequence does not end with a dedicated digital end marker. The normal
`rtrh_decoder::loop()` considers the measurement complete after 15 seconds with
no new edge:

```cpp
MEASUREMENT_QUIET_US = 15000000;
```

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

`BusSniffer` then adds the thermal-transient state and publishes the result to
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
web-server/task context, never in the ISR.

---

# Safe modification checklist

Before changing ISR-related code, check all of the following:

1. Does the ISR still perform only fixed-cost, non-blocking work?
2. Did any logging, allocation, BLE, web, flash or ESPHome publishing enter the
   ISR path?
3. Are phase boundaries still elapsed-time based?
4. Are G10 periods still measured only from a physical G10 IRQ?
5. Can a G10 period accidentally cross a REF/RT/RH phase boundary?
6. Is the first CO₂ bus state before the first captured edge still preserved?
7. Is CO₂ decoding still done only after capture is stopped?
8. Is RT/RH snapshot copying still protected by disabling/rechecking the two
   RT/RH GPIO interrupts?
9. Are ISR-shared fields still fixed-size and allocation-free?
10. Have both `debug_capture: false` and `debug_capture: true` builds been tested?
11. After flashing, do CO₂, RT/RH quality and MyAmbience history still behave
    normally?

If a change affects timing constants, pin assignments, state masks, sample
limits, buffer sizes, or ISR synchronization, treat it as a protocol-level
change rather than a cosmetic refactor and validate it against raw captures.
