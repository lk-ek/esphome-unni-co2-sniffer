# RT/RH high-humidity diagnostics (2026-08-17)

At about 67 %RH on the Unni display, the historical RH-state decoder stopped
seeing the characteristic `RT=0, RH=1` (`0x08`) state even though REF/RT timing
remained usable. USB operation with Light-sleep disabled reproduced the failure,
which rules out the battery Light-sleep policy as the direct cause.

This diagnostic revision does not change humidity decoding. It adds independent
observability for the RH phase:

- mean RT-derived carrier period during the RH phase and cycle count;
- carrier/REF ratio;
- RT rising/falling edge counts during RH;
- RH rising/falling edge counts during RH;
- the same values in the UDP timing payload, collector `rtrh_timing.csv`, and
  `/rt_rh_timing.csv` HTTP output.

The goal is to determine whether the RH carrier or relative edge timing remains
stable when the combined `0x08` state becomes too short or disappears. No new
RH calibration/fallback is enabled until captures establish a reliable mapping.

## Direct RT↔RH phase timing follow-up

The carrier/edge-count diagnostic showed that at about 69 %RH both RT and RH
continue to toggle at almost identical rates even though the historical
`RT=0, RH=1` state disappears. This points to loss of observable relative phase,
not loss of the RH oscillator itself.

The next diagnostic records corresponding rising and falling RT/RH edges as
one-to-one pairs during the RH phase. For each polarity it retains signed
`RH timestamp - RT timestamp` samples and reports their medians and a combined
mean. The rising-edge median is the primary calibration diagnostic and is also
normalized by the RH carrier period. Pairing is limited to 35 us, well below
half of the ~106 us carrier period observed near 68-69 %RH, so unrelated
neighboring carrier cycles are not joined. Positive values mean RH follows RT;
negative values mean RH leads RT.

These phase values are diagnostic only. They are exported in the UDP timing
payload, collector `rtrh_timing.csv`, HTTP `/rt_rh_timing.csv`, and serial log,
but they do not yet alter humidity validation, calibration, or publication.

## Pairing refinement after the first direct-phase capture

At an Unni display reading of about 18.1 °C / 68 %RH, three captures showed a
stable rising-edge phase median of +6 us while the falling-edge median was less
stable (-10, -10, +7 us). The RH carrier was about 105.6-106.0 us. This makes
the rising-edge phase the better provisional calibration observable.

The pairing window is therefore reduced from 70 us to 35 us, safely below half
a carrier period in this observed range. The normalized phase diagnostic now
uses the rising-edge median (`rh_phase_rise_us / rh_carrier_period_us`) rather
than the combined rising/falling mean. Falling-edge and mean values continue to
be exported for comparison but are not treated as primary calibration inputs.
Humidity publication still uses the historical decoder; this remains a
diagnostics-only change until enough RH points exist for a new mapping.
