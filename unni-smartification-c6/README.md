# Unni CO2 Smartification — ESP32-C6 Rev A v29 validated

Target: **KiCad 10.0.5**.

This package contains the completed schematic and KiCad/ngspice system simulation for the current Rev-A architecture. PCB work is intentionally excluded; the project focuses on a validated, editable schematic and repeatable power-path simulation.

## Sheets

- `01_usb_power.kicad_sch` — USB-C, MCP73831 charger, USB/battery load sharing, TPS63031 3.3 V rail, TPS613222A forced-awake 5 V rail, hardware USB inhibit, Unni AC/DC outputs.
- `02_esp_aux.kicad_sch` — ESP32-C6-MINI-1, native USB pair, ADC sensing, battery NTC, passive CO2/RT/RH taps, touch emulation, backlight sensing.
- `03_simulation_harness.kicad_sch` — simulation-only stimuli and loads; excluded from BOM/PCB.

## Key corrections in v29

- D1 corrected to conduct `VBUS -> SYS`, not `SYS -> VBUS`.
- TPS63031 FB connected to VOUT/+3V3.
- CHARGE_STAT divider R12, UNNI_AC capacitor C9, BAT_ADC C20, BAT_TEMP_ADC C21, and optional BACKLIGHT_ADC capacitor C33 are electrically connected.
- Forced-awake Q2/Q3/Q4 wiring validated from KiCad's exported netlist.
- VBUS pull-down R13 changed to 47 kΩ to prevent SS14 reverse leakage from falsely biasing Q4 while the internal boost is active.
- Simulation USB hot-plug is now open-circuit when absent, using `USB_HOTPLUG.lib`, rather than a 0 V source clamping VBUS.
- TPS613222A behavioral model now includes an equivalent input load/discharge path, so `BOOST_IN` and +5V collapse when Q2 opens. The obsolete I3 boost input proxy is excluded.
- The KiCad-exported 10 s transient now checks all five phases: idle battery, forced-awake, USB insertion with BOOST_CMD still high, USB removal, and forced-awake restart.

## Simulation in KiCad

Open `unni-smartification-c6.kicad_pro`, then **Inspect -> Simulator**. `unni-smartification-c6.wbk` contains four transient views. See `SIMULATION.md` for model fidelity and test details.

## Reproducible validation

```sh
KICAD_CLI=/path/to/kicad-cli \
NGSPICE_LIB=/path/to/libngspice.so \
./tools/validate_kicad.sh
```

Against the supplied KiCad 10.0.5 Linux AppImage environment, validation performs:

1. full schematic load and PDF export;
2. KiCad XML and SPICE netlist export;
3. ERC classification;
4. 41 critical pin/net assertions plus MCP73831 PROG verification;
5. four standalone ngspice regression decks;
6. a 10 s transient created from the SPICE netlist exported by KiCad itself;
7. phase-by-phase checks of Q2/Q3/Q4 inhibit, 3V3 regulation, USB handover, +5V shutdown, and forced-awake restart.

The headless AppImage reports only `lib_symbol_issues` and `footprint_link_issues` because it does not have a normal desktop user's global library tables. No electrical ERC categories remain.

## Model limits

AO3400A uses the selected detailed user-supplied MOSFET model. AO3401A, MCP73831, TPS63031 and TPS613222A are system-level behavioral models. They validate topology, source handover, charger behavior, enable/inhibit states and supply transients. They do **not** predict switch-node ripple, control-loop stability, switching losses or EMI.
