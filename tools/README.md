<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Reverse-engineering tools

These utilities are not part of the ESPHome firmware build.

- `capture2vcd.py` converts `LA02` and historical `LA01` I²C captures to VCD for waveform inspection.
- `blescan/` contains small macOS BLE/GATT inspection utilities used while reverse-engineering Sensirion compatibility.

The production firmware lives in `co2_monitor_0601/` and the top-level ESPHome YAML files.

## Unni UDP debug collector

`unni_debug_collector.py` receives the lightweight UDP capture stream emitted by the debug firmware and archives RT/RH and I2C captures without running an HTTP server on the ESP32-C3.

```bash
python3 tools/unni_debug_collector.py \
  --listen 10.0.42.149 \
  --port 45678 \
  --output ./unni-debug
```

### Segmenting capture sessions

While `unni_debug_collector.py` is running in an interactive terminal, press any key to create a ZIP archive containing every completed I2C and RT/RH capture received since the collector started or since the previous keypress. The archive also contains only the RT/RH timing rows from that same interval plus a small `capture_batch.txt` manifest. Capture collection continues after the archive boundary; `Ctrl-C` stops the collector.
