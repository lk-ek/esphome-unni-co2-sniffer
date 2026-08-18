# Current-hardware CO2 I2C corpus

Derived from `unni-captures-20260817-141938-135658.zip`. The raw GPIO-edge CSV files were independently decoded for START/address/data/ACK events, and only read frames at address `0x62` with the observed ACK pattern and a valid Sensirion CRC-8 (`init=0xFF`, polynomial `0x31`) were retained.

- Raw I2C CSV captures inspected: 331
- CRC-valid CO2 response frames retained: 86
- Observed CO2 range: 472..1033 ppm

This is a regression/provenance corpus, not an independent CO2 accuracy reference.
