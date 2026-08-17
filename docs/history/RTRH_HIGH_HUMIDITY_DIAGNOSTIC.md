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
