# RESOLVED: D3/G11 is not required

The shadow test documented below was completed successfully. The production decoder now uses only G10/D1 and G13/D2; D3/GPIO5 became free for other uses and is now used for USB/VBUS detection. This file is retained as historical validation context.

# RT/RH D3/G11 optionality shadow test — 2026-08-12

This build keeps the validated three-pin RT/RH decoder unchanged and adds a
shadow RH-period decoder that behaves as if G11/D3 did not exist.

The shadow decoder:

- receives state transitions only on physical G10/G13 interrupts,
- projects the state to G10 + G13,
- measures entries into `G10=0, G13=1`,
- does not affect temperature, humidity, validation, quality, HA, or BLE output.

In the debug build, `/rt_rh_timing.csv` adds:

- `state_rh_2pin_median_us`
- `state_rh_2pin_samples`
- `state_rh_2pin_seen`

Compare `state_rh_2pin_median_us` with the existing
`state_rh_median_us`. If they agree over the operating range with comparable
sample stability, G11/D3 can be made optional without changing the calibrated
RH observable.

The existing stored debug captures are decimated, so they cannot exactly replay
the ISR with G11 removed; this on-device shadow measurement is the lossless test.

## Result

Validation capture set `rh_th_nod3_260811-1.zip` produced:

| Capture | 3-pin median | 2-pin median | 3-pin samples/seen | 2-pin samples/seen |
|---|---:|---:|---:|---:|
| 1 | 687.000 µs | 687.000 µs | 96 / 189 | 96 / 189 |
| 2 | 689.000 µs | 689.000 µs | 96 / 189 | 96 / 189 |
| 3 | 686.000 µs | 686.000 µs | 96 / 190 | 96 / 190 |

The two-pin path therefore observed the same recurrence events and produced the
same median in all validation captures. Production code was changed to use only
G10/D1 and G13/D2; G11/D3 was removed from GPIO configuration and ISR handling.
