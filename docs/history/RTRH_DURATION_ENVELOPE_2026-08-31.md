<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# RT/RH native burst-duration envelope — 2026-08-31

After hardening Light-sleep wake synchronization, a battery-powered debug run
captured 31 complete, non-overflowing RT/RH waveforms from the final two-wire,
10 kohm tap installation. Their decimated raw traces ended with approximate RH
spans of 115.8..131.1 ms; 16 were below the historical 127 ms lower limit.

The shorter bursts were not ISR truncation artifacts. Their REF and RT phase
durations remained near 125 and 127 ms, RT/RH edge counts stayed balanced, and
the RH phase retained roughly 550..670 coherent carrier cycles. Adjacent
accepted and rejected cycles also produced a continuous carrier/REF humidity
observable. The fixed 127 ms lower bound was therefore rejecting complete
native bursts.

The lower acceptance limit was moved to 110 ms, leaving about 5.8 ms below the
shortest observed full burst. The 134 ms upper limit and the independent REF,
RT, carrier-count, carrier-period and carrier/REF plausibility checks were kept.
This changes capture acceptance only; calibration coefficients, phase timing,
ISR behavior and publication policy are unchanged.
