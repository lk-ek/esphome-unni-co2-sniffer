# Classic ESP32 ADC calibration compatibility

The standalone `mobilesensor-sensirion.yaml` targets a classic ESP32
(`wemos_d1_mini32`). ESP-IDF 5.5 exposes line-fitting ADC calibration on this
target, while the ESP32-C3/C6 Unni builds use curve fitting.

`CO2Monitor0601::setup_battery_adc_()` is compiled even when the standalone
mode does not instantiate battery entities, so the calibration code must remain
buildable for both target families. The implementation therefore selects
`adc_cali_create_scheme_line_fitting()` for `CONFIG_IDF_TARGET_ESP32` and keeps
the existing curve-fitting path for the C3/C6 builds.
