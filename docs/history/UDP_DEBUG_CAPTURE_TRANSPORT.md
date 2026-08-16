# UDP debug capture transport

The debug build can export raw capture data to a lightweight UDP collector instead of requiring ESPHome's `web_server`.

## Debug build defaults

`i2c-sniffer-debug.yaml` uses:

```yaml
logger:
  level: DEBUG
  initial_level: DEBUG

captive_portal:

co2_monitor_0601:
  debug_capture: true
  debug_udp_host: 10.0.42.149
  debug_udp_port: 45678
```

The debug build intentionally does not enable `web_server`.

## Collector on macOS

Run from the repository root:

```bash
python3 tools/unni_debug_collector.py --listen 10.0.42.149 --port 45678 --output ./unni-debug
```

The collector creates a date directory and archives:

- `i2c-*.la`: the compact LA02 waveform format used by the existing capture tooling.
- `i2c-*.csv`: decoded SCL/SDA edge samples for quick inspection.
- `rtrh-*.csv`: RT/RH raw edge captures using the current GPIO3/GPIO4 names.
- `rtrh_timing.csv`: one decoder/timing summary row per RT/RH measurement.

The wire protocol uses UDP datagrams with a small binary `UND1` header and a payload of at most 1000 bytes, avoiding IP fragmentation on normal Ethernet/Wi-Fi MTUs. Multi-packet captures carry capture ID, packet index and packet count. The collector reports incomplete captures when packets are missing.

No ACK/retry protocol is used. This is deliberate: debug export must not introduce persistent connections, large response buffers or retransmission state on the ESP32-C3.

## Runtime behavior

- UDP is never sent from a GPIO ISR.
- RT/RH raw data is sent incrementally from normal component-loop context, one datagram per loop iteration.
- I2C raw captures are exported immediately after a complete quiet-period-delimited capture, before the ISR buffer is reused.
- When UDP export is active, I2C raw captures are not duplicated into the old heap-backed HTTP snapshot string.
- Normal Home Assistant data continues to use the ESPHome Native API.


## Socket initialization

The UDP destination is configured during component setup, but the lwIP socket is opened lazily on the first capture packet from the normal loop. This avoids using lwIP before ESPHome has initialized Wi-Fi/TCP-IP.

### Retry/backpressure follow-up

The UDP exporter now treats network unavailability and transient lwIP allocation
failures as backpressure rather than capture loss. It waits until the station is
associated, spaces datagrams by at least 5 ms, keeps multi-packet I2C captures
frozen in the existing ISR buffer until all fragments have been sent, and keeps
RT/RH timing payloads pending until they can be transmitted. No UDP send occurs
from an ISR.
