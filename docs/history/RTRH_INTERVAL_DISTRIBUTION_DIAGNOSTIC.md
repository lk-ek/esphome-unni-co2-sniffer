<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# RT/RH interval distribution diagnostic

Temporary diagnostic added while investigating intermittent RH decoding where the retained
state-period median alternated between roughly 220 us and roughly 440 us.

The decoder now derives, from the already-retained RH-state interval buffer only:

- minimum, 25th percentile, median, 75th percentile and maximum interval;
- count of intervals in 160..280 us (the observed ~220 us population);
- count of intervals in 360..520 us (the observed ~440 us population);
- count of all remaining intervals.

The summary is emitted once per completed RT/RH measurement. No ISR work, acceptance limits,
median selection, temperature/humidity calculation, entity publication, BLE behaviour, or HTTP
capture behaviour is changed by this diagnostic.
