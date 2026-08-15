<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Development process and history

This document summarizes how the Unni CO₂ Sensor Smartification project evolved and why the current architecture looks the way it does. It is intentionally historical: for current wiring and configuration, use the project [`README.md`](../README.md), [`ISR_ARCHITECTURE.md`](ISR_ARCHITECTURE.md), and [`LIGHT_SLEEP.md`](LIGHT_SLEEP.md).

Detailed experiment notes are retained under [`docs/history/`](history/).

## 1. Initial goal and passive-sniffer approach

The project started with a simple constraint: add ESPHome/Home Assistant and Sensirion-compatible BLE to the existing Unni CO₂ monitor without replacing or taking control of the original electronics.

The ESP therefore acts as a passive observer. It decodes:

- the original CO₂ module's two-wire bus,
- the timing signals used by the Unni PCB for its temperature and humidity sensor positions,
- and later the battery and USB/VBUS state.

Keeping the original Unni MCU in charge proved useful because the display, calibration behavior, battery detection, and backlight behavior remain native to the device.

## 2. ESP32-S2 prototype and migration to XIAO ESP32-C3

Early reverse-engineering work used an ESP32-S2 and signal names such as `G10` and `G13`, derived from the prototype GPIO assignments. Those names survived for a while even though they were not names from the Unni PCB.

The project later moved to the Seeed Studio XIAO ESP32-C3 for its smaller size, lower power use, BLE support, and easier integration inside the sensor.

The final signal terminology follows the actual purpose of the Unni PCB points:

- `RT` — temperature-sensor position / timing signal,
- `RH` — humidity-sensor position / timing signal,
- CO₂ SDA,
- CO₂ SCL.

The current default C3 mapping is documented in the main README.

## 3. RT/RH reverse engineering

The RT/RH interface is not a normal digital temperature/humidity protocol. The Unni electronics measure timing relationships around the unpopulated RT and RH sensor positions.

The decoder evolved from raw edge capture into a phase-based measurement pipeline:

1. identify the stable reference timing,
2. measure the RT phase,
3. identify the recurring RH state,
4. reject malformed captures,
5. derive temperature and humidity from normalized timing ratios.

Typical reference periods were around 77 µs. RT and RH measurements were normalized against this reference to reduce dependence on absolute MCU timing.

A major part of the development process was collecting captures over different temperatures and humidity levels and fitting calibration functions against the Unni display and external reference measurements.

The calibration iterations and validated ranges are preserved in:

- [`history/CALIBRATION_2026-08-11.md`](history/CALIBRATION_2026-08-11.md)
- [`history/CALIBRATION_2026-08-11_v2.md`](history/CALIBRATION_2026-08-11_v2.md)
- [`history/CALIBRATION_2026-08-11_v3.md`](history/CALIBRATION_2026-08-11_v3.md)
- [`history/CALIBRATION_2026-08-11_v4.md`](history/CALIBRATION_2026-08-11_v4.md)

## 4. Measurement quality and diagnostics

Raw captures showed that a decoded value alone was not enough: corrupted or transitional cycles needed to be distinguishable from good measurements.

The decoder therefore gained:

- reference, RT and RH timing diagnostics,
- normalized timing ratios,
- measurement quality scoring,
- structured plausibility checks,
- thermal-transient detection,
- calibration extrapolation indicators.

These were heavily used during development but are optional in normal operation. Current builds create them automatically only when `debug_metrics: true` is enabled.

Historical details:

- [`history/DEBUG_AND_QUALITY_V5.md`](history/DEBUG_AND_QUALITY_V5.md)
- [`history/QUALITY_REFACTOR_V7.md`](history/QUALITY_REFACTOR_V7.md)
- [`history/DEBUG_CAPTURE_OPTION.md`](history/DEBUG_CAPTURE_OPTION.md)

## 5. Reducing RT/RH from three signals to two

For a period the RT/RH decoder also sampled a third signal on D3/GPIO5. The RH state was originally identified using a three-line state combination.

A shadow decoder was then added that ignored the third line while processing the same live measurements. Across the recorded validation samples, the two-pin decoder produced exactly the same RH state median, sample count, and observed-state count as the three-pin decoder.

The third line was therefore removed from the production decoder with no observed loss of measurement quality. This freed D3/GPIO5 for later USB/VBUS detection.

The experiment and result are preserved in [`history/RTRH_D3_OPTIONAL_TEST.md`](history/RTRH_D3_OPTIONAL_TEST.md).

## 6. CO₂ bus sniffing

The CO₂ channel is decoded passively from its SDA/SCL traffic. The decoder evolved toward a deliberately narrow implementation: it only recognizes the frame structure needed to extract the CO₂ value and rejects partial or malformed transactions.

Power-management testing later revealed that collecting partial CO₂ traffic while the CPU was moving in and out of Light Sleep caused spurious frame errors. The final design therefore enables CO₂ capture only during an active measurement window and resets partial capture state at the window boundary.

## 7. Sensirion-compatible BLE and MyAmbience

The project emulates a Sensirion `T_RH_CO2_ALT` gadget so the Unni measurements can be consumed by Sensirion MyAmbience.

Development included:

- matching the Sensirion manufacturer-data payload,
- preserving the short GAP name `S`,
- implementing the required GATT services,
- adding persistent history storage and download,
- handling CCCD subscribe/unsubscribe behavior,
- dealing with advertising/GATT races and connection lifecycle issues.

The iPhone MyAmbience app became the most useful end-to-end interoperability test: successful discovery, connection, live values, and full history download validate most of the BLE stack at once.

macOS MyAmbience exposed additional discovery/caching quirks. One regression was traced to ESPHome advertising the node name `i2csniffer` instead of the Sensirion-compatible `S`; the BLE component now sets the GAP name before ESPHome initializes the BLE stack.

Relevant notes:

- [`history/BLE_GATT_RELIABILITY_FIX.md`](history/BLE_GATT_RELIABILITY_FIX.md)
- [`history/BLE_OPTIONS.md`](history/BLE_OPTIONS.md)
- [`history/REFACTOR_V11.md`](history/REFACTOR_V11.md)
- [`history/REFACTOR_V12.md`](history/REFACTOR_V12.md)

## 8. Persistent BLE history

History originally grew as a separate subsystem with its own sample encoding and state. It was later consolidated so live BLE and history use the same encoded Sensirion sample format.

The current implementation uses a RAM ring plus an append-oriented flash journal in the `senshist` partition. The partition is requested by the component automatically when history is enabled, so users no longer need to maintain a custom partition table in YAML.

The preserved design history is in [`history/REFACTOR_V12.md`](history/REFACTOR_V12.md).

## 9. ESPHome component refactoring

The early implementation concentrated capture, decoding, quality checks, publishing, BLE and history logic in a small number of large files. Several refactors progressively separated responsibilities:

- calibration model,
- RT/RH decoder,
- CO₂ decoder,
- BLE live advertising,
- BLE history,
- power management,
- ESPHome configuration/code generation.

The configuration layer was also moved toward component-owned defaults. Today the component automatically provides its main entities, BLE server setup, history partition, power-management sdkconfig, and default pin mapping.

This dramatically reduced user YAML while keeping overrides available.

Refactor notes:

- [`history/REFACTOR_V6.md`](history/REFACTOR_V6.md)
- [`history/CONFIG_CODEGEN_REFACTOR.md`](history/CONFIG_CODEGEN_REFACTOR.md)
- [`history/REFACTOR_V10.md`](history/REFACTOR_V10.md)
- [`history/REFACTOR_V11.md`](history/REFACTOR_V11.md)
- [`history/REFACTOR_V12.md`](history/REFACTOR_V12.md)
- [`history/REFACTOR_V13.md`](history/REFACTOR_V13.md)

## 10. Power-saving development

Power consumption became a major part of the project once battery operation was added.

Several approaches were tested:

### Lower CPU frequency

The ESP32-C3 was reduced from the usual 160 MHz operating point to an 80 MHz maximum. This substantially reduced heating while leaving the timing decoder reliable.

### Wi-Fi and Home Assistant throttling

Home Assistant updates do not need to follow every decoded bus transaction. Publication was therefore throttled to roughly one update per RT/RH measurement cycle, while internal decoding continues at the native signal cadence.

### Automatic Light Sleep

The final design uses ESP-IDF automatic Light Sleep rather than reboot-oriented Deep Sleep.

RT and RH GPIO activity wakes the C3. Once awake, the component acquires both a no-Light-Sleep lock and a maximum-CPU-frequency lock, completes the RT/RH measurement, waits for one valid CO₂ frame, publishes the result, and releases the locks again.

The CO₂ bus does not wake the CPU by itself.

Early Light-Sleep experiments exposed two important issues:

- waiting too long for the RT/RH decoder to declare a measurement complete wasted most of the sleep opportunity,
- allowing the CPU to run at the lower DFS frequency during capture caused repeated CO₂ frame errors and 10-second awake timeouts.

The final capture window therefore uses 80 MHz and normally lasts only a few seconds.

See [`LIGHT_SLEEP.md`](LIGHT_SLEEP.md) and the historical power-save notes:

- [`history/POWERSAVE_V8.md`](history/POWERSAVE_V8.md)
- [`history/POWERSAVE_V9_EXPERIMENTAL.md`](history/POWERSAVE_V9_EXPERIMENTAL.md)
- [`history/POWERSAVE_V9_1_PRODUCTION.md`](history/POWERSAVE_V9_1_PRODUCTION.md)

## 11. BLE power consumption and modem sleep

Initial measurements showed that simply enabling BLE increased total power dramatically, even with relatively slow advertising. Disabling BLE reduced consumption to roughly the ~85 mW range in the tested setup, while the initial BLE build was around ~330–340 mW.

The main improvement came from enabling the ESP32-C3 Bluetooth controller's modem-sleep path and PHY/MAC/baseband power-down. After that change, measured total power dropped to roughly:

- ~107 mW with a 5-second advertising interval,
- ~125 mW with a 2-second advertising interval.

The 2-second interval was chosen as the default because 5 seconds noticeably slowed MyAmbience discovery/connection, while 2 seconds still retained most of the modem-sleep savings.

These values are measurements of the complete tested device setup, not ESP32-C3 datasheet currents.

## 12. Battery operation

The Unni board has separate USB/AC and battery/DC power inputs. Testing confirmed that a single-cell Li-ion/LiPo source on the battery path powers both the original sensor and the integrated ESP, and the Unni itself correctly recognizes battery operation by turning off its backlight after a few seconds.

A 650 mAh, 3.7 V cell was used for runtime estimates during development. At the optimized BLE power level, expected useful runtime is on the order of tens of hours rather than the few hours implied by the initial BLE implementation.

### Battery voltage measurement

D0/GPIO2 is connected to the midpoint of a 1 MΩ / 1 MΩ divider across the battery, with 0.1 µF from the ADC node to ground.

The firmware uses calibrated ESP32-C3 ADC1 measurements and publishes:

- `Battery Voltage`,
- `Battery Level` as a voltage-derived Li-ion SOC estimate.

A significant discovery was that the battery/charger node can sit near 4.2 V while USB is present even when no battery is physically connected. Battery voltage alone therefore cannot be used as a reliable battery-presence detector.

### USB/VBUS detection

D3/GPIO5 was repurposed after the third RT/RH line was removed. It now monitors VBUS through a 220 kΩ / 220 kΩ divider and publishes `USB Power`.

When USB power is present, `Battery Level` is marked unavailable because charger-held voltage is not a useful open-circuit SOC estimate. `Battery Voltage` remains available as a raw electrical measurement.

A brief false-negative during initial testing turned out to be a soldering fault rather than a problem with the 220 kΩ / 220 kΩ divider.

## 13. USB-to-battery switchover behavior

The device was observed to remain running reliably when going from battery to USB, but it can occasionally reset when USB is unplugged and the battery path takes over.

This points to a short supply dip during USB-to-battery switchover rather than a firmware-triggered restart. The Unni electronics can continue operating through a disturbance that is still large enough to brown out the ESP32-C3.

This remains primarily a hardware power-path consideration; bulk capacitance close to the ESP supply is a potential mitigation if needed.

## 14. Debug-capture HTTP crash

During decoder validation, downloading timing/capture data could crash the ESP32-C3 in newlib `_dtoa_r`. The failure was traced to a large `snprintf()` path formatting many floating-point values in the HTTP handler.

The handler was changed to avoid `%f` formatting in that path and to send bounded chunks/lines instead of relying on one large formatted buffer. The measurement decoder itself was unaffected by that crash.

This incident is one reason current debug paths deliberately avoid unnecessarily expensive formatting in timing-sensitive builds.

## 15. BLE server initialization ordering

When `esp32_ble` and `esp32_ble_server` were moved out of user YAML and into component-managed setup, an initialization-order bug caused a boot-time load-access fault in `BLEServer::create_service()`.

The cause was creating GATT services before ESPHome had finished wiring the BLE server's parent object. The component now stores the generated server pointer during code generation and defers actual GATT service creation until `CO2Monitor0601::setup()`.

This preserved the simplified YAML without relying on fragile initialization timing.

## 16. Automatic entity registration

Another side effect of moving sensor definitions into the component was that sensor objects existed and received `publish_state()` calls, but ESPHome Web and Home Assistant reported no sensors.

The missing part was the ESPHome entity-domain compile definition. The component now explicitly enables `USE_SENSOR` and, when debug metrics are active, `USE_BINARY_SENSOR` so automatically created child entities are visible through the native API and web interface.

## 17. ESPHome version compatibility

The component was developed against ESPHome 2026.7.x and then tested during the 2026.8 beta cycle. The move toward component-owned setup intentionally uses ESPHome's supported mechanisms for sdkconfig options, partition requests, child entities, and BLE components rather than maintaining large amounts of duplicated YAML.

Because the Sensirion emulation still uses relatively low-level ESPHome BLE-server APIs, BLE remains the area most likely to require maintenance if ESPHome changes its internal BLE architecture in a future release.

## 18. Current design principles

The development process converged on a few principles that are useful when changing the project further:

1. **Remain passive toward the Unni electronics.** The ESP should observe signals, not become necessary for the original device to operate.
2. **Keep timing-critical ISR work minimal.** Decode and publishing belong outside interrupt context.
3. **Normalize measurements against the Unni reference timing.** This was more stable than relying only on absolute periods.
4. **Prefer live A/B tests over assumptions.** The third RT/RH pin, BLE power use, and Light-Sleep behavior were all resolved through direct measurements.
5. **Keep user YAML small.** Hardware-specific requirements and safe defaults belong in the component when ESPHome provides a supported mechanism for doing so.
6. **Treat voltage-derived battery percentage as an estimate.** Charging state and load materially affect cell voltage.
7. **Keep detailed failed experiments.** They prevent future work from repeating already-tested approaches.



## 2026-08-14: VBUS-aware runtime power policy

After battery and VBUS sensing were available, the power strategy was split by power source rather than applying the same low-power behavior all the time. USB operation now keeps the ESP32-C3 awake at 80 MHz and captures the CO2 bus continuously, while battery operation retains the RT/RH-triggered Light-Sleep capture window. Home Assistant receives fresh measurements immediately on USB but only the latest cached sensor set once per minute on battery. Sensirion-compatible BLE advertising also changes dynamically from 2 seconds on USB to 5 seconds on battery. This preserves responsiveness when external power is available without giving up the measured battery savings.

## 2026-08-14: Generic passive I²C framing layer

The CO₂ path was split into a generic passive `i2c_sniffer` and a protocol-only
`co2_decoder`. GPIO edge capture, START/repeated-START/STOP detection, 7-bit
address/direction decoding, ACK/NACK reconstruction, capture enable/disable and
the raw `/capture` endpoint now live in the generic layer. The CO₂ decoder only
claims the observed 0x62 / 0xEC05 exchange, validates its CRC and produces ppm.
Debug-capture builds additionally log every reconstructed I²C frame that is not
claimed by the CO₂ decoder.

## 2026-08-14: Component renamed for device identity

The top-level ESPHome integration was renamed from the generic historical `bus_sniffer` name to `co2_monitor_0601`, matching the tested device designation **CO2 Monitor Carbon Dioxide Detector 0601**. The C++ namespace and orchestrator class are now `co2_monitor_0601` and `CO2Monitor0601`; the component source files were renamed accordingly. This is intentionally a breaking configuration rename: YAML now uses `co2_monitor_0601:` rather than keeping a legacy alias. Internal protocol modules such as `i2c_sniffer`, `co2_decoder`, and `rtrh_decoder` retain their descriptive names.


## 2026-08-14: I²C frame validity and suspicious-capture freeze

Debug logging of newly generalized I²C traffic exposed occasional reconstructed frames that looked like unrelated slave addresses or one-byte commands. Correlation with the decoded CO₂ values showed that some were fragments of normal 0x62 traffic after missed or ambiguously segmented edges. The generic framer now assigns a structural `FrameStatus` and separates malformed captures from valid-but-unhandled I²C frames. Malformed frames are counted once and are never offered to the CO₂ protocol decoder. Structurally valid CO₂ frames that fail CRC, ACK, or length checks are also treated as suspicious for debug capture purposes. Debug builds freeze the first suspicious raw I²C capture until `/capture` is downloaded, making the actual edge sequence available before later normal traffic can overwrite it. The raw format was advanced to `LA02` to preserve the pre-first-edge bus state, and `capture2vcd.py` was updated to read both `LA02` and historical `LA01` captures.

## 2026-08-14 — Debug capture HTTP reliability

- Stream I2C `/capture` and RT/RH `/rt_rh_capture.csv` downloads in ~1 KiB chunks.
- Preserve frozen/snapshotted debug data when an HTTP transfer fails or the client disconnects.
- Release an I2C suspicious-capture freeze only after a complete successful transfer.
- Reduce RT/RH CSV HTTP chunk count and log ESP-IDF send errors with retry semantics.

## 2026-08-14 — RMT-assisted I²C SCL recovery

Two frozen VCD captures isolated separate GPIO-capture failure modes. In one,
an SDA setup edge and following SCL rise were collapsed into a single ISR
sample; in the other, a complete SCL high/low pulse was absent from the GPIO
trace even though the CO₂ sensor had received the command. The generic I²C
sniffer now resolves the first case contextually and uses an ESP32-C3 RMT RX
channel as an independent SCL pulse timeline for the second. RMT repair is
strictly conservative and falls back to GPIO-only decoding whenever the two
timelines cannot be aligned safely. The original GPIO waveform is still frozen
for debug download whenever a repair is used. RMT RX ISR handling is made
cache-safe, internal pulls are restored to disabled after RMT allocation, and
the RMT channel follows the existing USB/battery capture policy.


## 2026-08-14 — RMT transaction arming and async capture download

Field logs showed that the first RMT-assisted build never reported an actual
RMT recovery. The RX job had been armed while SCL was already idle; its 5 ms
RMT idle threshold therefore completed the job long before the next ~6 s CO₂
transaction. RMT RX is now armed from the first GPIO edge of each capture, with
`rmt_receive()` placed in IRAM as required for ISR use. RMT timestamps are
aligned absolutely and the artificial t=0 edge from the first RMT pulse was
removed. Coalesced-edge-only captures no longer consume the frozen debug slot.
The `/capture` endpoint now uses ESP-IDF asynchronous-request handling so a slow
or stalled download cannot occupy the main HTTP server task; failed transfers
still preserve the frozen capture for retry.


## 2026-08-14 — RMT isolated after Wi-Fi regression

Field A/B testing showed stable Wi-Fi with the pre-RMT firmware and flaky Wi-Fi/API
with RMT-enabled builds. RMT SCL assistance is therefore experimental opt-in and
disabled by default; normal builds do not link/configure RMT. The shared GPIO ISR
service is now installed early with `ESP_INTR_FLAG_IRAM` while preserving the
configured signal-pin isolation delay.

## 2026-08-14 — Protocol-validated missing-clock recovery and RMT/Wi-Fi follow-up

Further GPIO-only VCD captures showed a repeatable one-clock slip: a complete
SCL pulse can disappear from the GPIO trace while the real CO₂ sensor still
receives the transaction. The generic I²C layer now proposes single-clock
reconstructions only in suspicious timing gaps. A reconstruction is accepted
only when exactly one candidate passes a strict CO₂ transaction validator
(`0x62`, `EC05`, ACK/NACK pattern and Sensirion CRC); ambiguous or merely
plausible timing is never enough.

The earlier RMT diagnosis was corrected after checking ESP-IDF behavior: an RX
job may be armed while the line is idle because actual reception starts at the
first level change. The experimental RMT path therefore returns to task-context
pre-arming, reserves 96 symbols instead of the minimum 48 to avoid ping-pong
copy interrupts for the ~63-clock CO₂ burst, fixes interrupt priority at 1, and
no longer forces cache-safe RMT ISR / receive-in-IRAM options. RMT remains
disabled by default because field A/B tests associated it with Wi-Fi/API
instability on the single-core ESP32-C3.

The I²C `/capture` endpoint was also simplified back to a synchronous HTTPD
response. Real captures are typically under 1 KiB, so the separate asynchronous
FreeRTOS sender task added scheduling and request-lifetime complexity without a
useful throughput benefit. Frozen captures are still released only after a
successful transfer. Shipped Wi-Fi examples disable post-connect roaming for
this stationary device to avoid deliberate BSSID switches during diagnostics.

## 2026-08-14 — RMT retired; bounded two-clock GPIO recovery

Further field testing showed that RMT-enabled ESP32-C3 builds repeatedly degraded
Wi-Fi/API responsiveness, made small `/capture` sends block or time out, and in
one run ended with BLE `Malloc failed` / `Memory Full`. A previous RMT build also
aborted. In contrast, GPIO-only builds with the shared ISR service installed via
`ESP_INTR_FLAG_IRAM` kept networking stable and downloaded the same sub-kilobyte
captures synchronously. The RMT option, driver dependency, runtime path, and YAML
configuration have therefore been removed rather than retained as an experimental
feature. Historical entries above document the experiment but no longer describe
a supported configuration. Later driver inspection also showed that reserving 96
non-DMA RMT symbols did not eliminate the driver's RX ping-pong threshold path as
previously assumed.

A batch of frozen GPIO VCDs exposed why the first software recovery missed many
valid cases: it measured adjacent sample gaps, so an SDA transition could split an
otherwise obvious long constant-SCL interval. Recovery now measures HIGH/LOW SCL
level durations independently, preserves SDA transitions inside a hypothesized
missing pulse, and can test at most two missing pulses in different intervals.
Timing only proposes bounded candidates. Recovery is accepted only when every
protocol-valid candidate decodes to the same `0x62 W EC05` plus CRC-valid read
transaction; ambiguous results remain errors. Captures requiring one and two
missing clocks are covered by the field VCD regression set, while a trace that
ends after the command without any read edges remains intentionally unrecoverable.


## 2026-08-15 — Energy Save Mode measurement override

Added a Home Assistant `Energy Save Mode` switch plus `energy_save_mode_default`. The override preserves truthful physical USB/VBUS reporting but forces the runtime battery policy so USB power meters can compare normal and power-saving behavior on identical hardware. The switch affects PM locks/Light Sleep, CO2 capture gating, BLE advertising cadence, and Home Assistant publication cadence; battery SOC availability continues to follow physical VBUS.


### 2026-08-15: Energy Save transition grace and Wi-Fi MIN_MODEM

Changed shipped Wi-Fi power saving from HIGH/MAX_MODEM to LIGHT/MIN_MODEM after Energy Save Mode caused Wi-Fi loss when automatic MCU Light-sleep became active. Added configurable `energy_save_grace` (default 3 s) so a Home Assistant switch transition is visible before the USB PM locks are released and native USB Serial/JTAG may disconnect.

### 2026-08-15 — BLE-only zero-entity build fix

The BLE-only configuration keeps the shared sensor/binary-sensor/switch C++ type
framework available but intentionally registers zero ESPHome entities. ESPHome
2026.8 requires `ESPHOME_ENTITY_*_COUNT` for every enabled entity domain, so the
component now emits explicit zero counts when `home_assistant: false`. This does
not instantiate entities or enable Wi-Fi/API; it only makes the zero-entity
framework state explicit to core code generation.

### 2026-08-15 — Native API diagnostics stabilized

After restoring the known-good Home Assistant entity-registration path, both the
normal debug build and the SHT43 identity probe again exposed their entities to
Home Assistant. A subsequent API-isolation test showed that `VERY_VERBOSE`
logging together with an artificially reduced API connection limit could itself
make the ESP32-C3 less stable.

The diagnostic builds therefore use targeted `VERBOSE` logging for `api` and
`api.connection`, while keeping the initial logger level at `DEBUG`. The API
connection limit is left at ESPHome's default. Production and power-measurement
builds retain their lower logging levels. No Home Assistant codegen, BLE identity,
or power-policy behavior was changed as part of this stabilization.

## 2026-08-15: SHT43 Home Assistant ListEntities A/B test

Home Assistant could complete the encrypted native-API handshake and receive
`DeviceInfoResponse`, but the SHT43 probe intermittently stopped responding as
soon as Home Assistant sent `ListEntitiesRequest`. The device still had all
entities registered locally.

For isolation, the SHT43 probe now keeps the complete SHT43 identity, Device
Settings GATT table, security configuration and BLE advertising behavior but
does **not** register the experimental `BLE Pairing Mode` ESPHome switch. The
normal/debug builds retain that switch. A SHT43 probe boot should therefore
report one ESPHome switch instead of two. This is intentionally an A/B
diagnostic, not a final pairing UX decision.


## 2026-08-15: SHT43 Device Settings GATT A/B test

After the pairing-switch A/B build proved that Home Assistant could enumerate all
entities and receive `ListEntitiesDoneResponse`, the native API still stopped
responding immediately after `SubscribeStatesRequest`; even keep-alive pings were
left unanswered while the sensor main loop continued to run.

For the next isolation step, only the SHT43 identity-probe build omits the
experimental Sensirion Device Settings service (`0x8100`) and its GATT event
handler. The SHT43 identity, SHT/T/RH GATT services, BLE security, advertising,
history and measurement paths remain enabled. The previous omission of the
`BLE Pairing Mode` Home Assistant switch is retained so this test changes only
one additional subsystem. The probe also logs free 8-bit heap and the largest
free 8-bit block after BLE/GATT setup and every 30 seconds.

### 2026-08-15: SHT43 GATT heap isolation

For API/heap diagnosis, the SHT43 probe build can advertise the SHT43 identity
without creating the additional SHT, temperature, humidity, or Device Settings
GATT services. The established Sensirion history service remains enabled. Heap
logging records memory before BLE activation, on the first GATTS event, and
periodically at runtime.
