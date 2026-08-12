# Compile-time debug capture option

The hard-coded `RTRH_DEBUG_CAPTURE` switch is now exposed as an ESPHome option:

```yaml
bus_sniffer:
  debug_capture: false
```

Default: `false`.

## Production (`debug_capture: false`)

No `web_server:` component is required. The following debug-only pieces are not compiled/registered:

- `/capture`
- `/rt_rh_capture.csv`
- `/rt_rh_timing.csv`
- retained HTTP copy of the latest raw I2C capture
- RT/RH raw debug capture buffer
- HTTP capture handlers and their mutex

The live CO2 I2C capture/decoder remains active; its ISR sample buffer is required for decoding and is not a web/debug feature.

`debug_metrics` remains independent and can still be enabled for INFO/diagnostic output and HA diagnostic entities.

## Capture/debug build (`debug_capture: true`)

Add a top-level web server explicitly:

```yaml
web_server:
  port: 80

bus_sniffer:
  debug_capture: true
```

This restores all three capture endpoints.
