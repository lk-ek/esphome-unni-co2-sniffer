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
