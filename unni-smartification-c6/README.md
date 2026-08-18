# Unni CO2 Smartification — KiCad 10 Rev A v3

Target: KiCad 10.0.5.

This revision is deliberately a **single flat schematic**. The previous hierarchical draft hid parser errors and contained placeholder symbols without meaningful wiring. v3 prioritizes a schematic that KiCad itself loads and exports before re-introducing hierarchy.

## Implemented functional blocks

- ESP32-C6-MINI-1 using the KiCad `RF_Module` symbol and footprint.
- Native USB D-/D+ on GPIO12/GPIO13, with 22-ohm series resistors reserved close to the module.
- Replacement USB-C daughterboard with 5.1-kohm Rd on CC1/CC2.
- BQ24074 Li-ion charger / PowerPath.
- TPS63031 fixed 3.3-V buck-boost.
- TPS61023 switchable 5-V forced-awake boost.
- TPS2116 USB-priority power mux feeding UNNI_AC.
- Direct battery feed to UNNI_DC.
- Passive CO2 and RT/RH taps through 10-kohm series resistors.
- Touch-electrode emulator and backlight voltage sensing.
- Battery and raw USB ADC dividers.

Custom symbols are used only where KiCad 10.0.5 does not ship a suitable symbol (BQ24074, TPS61023, TPS2116).

## Important

This is still Rev A engineering work, not yet a fabrication release. Mechanical connector choices, charge current, boost load/current margin, and the real touch/backlight electrical measurements remain to be verified before layout.
