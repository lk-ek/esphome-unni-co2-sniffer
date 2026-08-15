# RT/RH RH-phase state diagnostic

Diagnostic A/B build for the SHT43 coexistence investigation.

The RT/RH decoder now records, during the fixed RH phase only:

- RT GPIO interrupt count,
- RH GPIO interrupt count,
- transitions into combined RT/RH states `00`, `01`, `08`, and `09`.

Rejected measurements also log the temperature estimate derived from RT/REF. This does not change acceptance, publication, calibration, BLE, or Home Assistant behavior.
