<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Reverse-engineering tools

These utilities are not part of the ESPHome firmware build.

- `capture2vcd.py` converts `LA02` and historical `LA01` I²C captures to VCD for waveform inspection.
- `blescan/` contains small macOS BLE/GATT inspection utilities used while reverse-engineering Sensirion compatibility.

The production firmware lives in `co2_monitor_0601/` and the top-level ESPHome YAML files.
