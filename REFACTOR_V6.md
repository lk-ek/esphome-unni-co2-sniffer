# v6 calibration refactor

The timing decoder and calibration model are now separated.

- `bus_sniffer.cpp`: capture, phase decoding, quality checks, diagnostics and publishing
- `bus_sniffer/calibration.h`: calibration coefficients, model functions and calibration envelope
- BLE/history modules remain unchanged

The v4 temperature and RH coefficients are unchanged. This is a structural refactor only.

Decoder-quality rules such as the 32-sample RH minimum remain in `bus_sniffer.cpp`; they describe capture validity rather than the sensor calibration.
