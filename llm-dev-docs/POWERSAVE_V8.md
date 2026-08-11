# v8 power-save profile

Built on v7 buildfix2. Calibration and measurement-quality logic are unchanged.

## Wi-Fi

- `power_save_mode: HIGH` (already the most aggressive ESPHome Wi-Fi modem-sleep mode)
- `fast_connect: true` to avoid a full active channel scan during normal reconnect/boot
- fallback AP and web server remain enabled because they are useful for service/capture

## CPU

- ESP32-C3 CPU frequency is set to **80 MHz** instead of the default 160 MHz.
- This is intentionally testable rather than hidden: watch RT/RH quality and reject reasons.
- If edge capture quality worsens, change `cpu_frequency` back to `160MHz`.

Automatic Light Sleep is deliberately **not** enabled. The sniffer depends on GPIO
edge interrupts and precise timing; ESP-IDF documents increased interrupt latency
with power-management frequency switching and peripherals being clock-gated in
Light Sleep.

## BLE

- actual GAP advertising interval: **5 s** instead of 2 s
- CO2 values are still sampled internally at the original cadence
- setters no longer stop/reconfigure/restart advertising on every CO2 frame
- the complete live BLE payload is committed once per valid RT/RH cycle (~30 s)
- each committed payload includes the latest available CO2 value

This removes the previous ~6-second BLE advertiser reconfiguration churn while
keeping continuous connectable advertisements and the GATT/history server alive.

## Home Assistant

- `ha_publish_interval` remains 30 s
- first values still publish immediately
- diagnostics remain available for testing

## CSV fix

The REF/RT rows now contain exactly the same 20 columns as the header/RH row. v7/v8 initially emitted one extra empty field on those two rows, causing CSV parsers to shift columns.
