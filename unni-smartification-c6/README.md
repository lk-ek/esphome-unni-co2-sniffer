# Unni CO2 Smartification — ESP32-C6 Rev A v29 validated

Target: **KiCad 10.0.5**.

This package contains the completed schematic and KiCad/ngspice system simulation for the current Rev-A architecture. Routing and component placement are still intentionally incomplete. The PCB file now contains the measured replacement USB-daughterboard mechanical stub (1.0 mm board thickness, fixed USB/top datum and mounting holes) so layout can proceed against real mechanics.

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

## Rev A v30: worst-case power regression

The validation suite now includes a 12-case battery sweep (4 OCV values x 3 internal resistances) with the ESP radio burst aligned to forced-awake startup. Run it through `tools/validate_kicad.sh`; detailed results are generated under `validation/worst-case/`.

## Portable ngspice validation (macOS / Linux)

The validation scripts no longer contain a hard-coded Linux AppImage path. `tools/run_ngspice_shared.py` resolves ngspice in this order:

1. `NGSPICE_LIB=/absolute/path/to/libngspice`
2. KiCad's macOS `KiCad.app/Contents/Frameworks`, Homebrew and common Unix library paths
3. `ctypes.util.find_library("ngspice")`
4. `NGSPICE_BIN` or an `ngspice` executable in `PATH`

A normal macOS invocation is simply:

```sh
KICAD_CLI="/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli" ./tools/validate_all.sh
```

If KiCad's bundled library is not discoverable, either set `NGSPICE_LIB` explicitly or install/use a standalone ngspice executable. The project itself does not require the author's `/mnt/data/...` paths.


## Mechanical reference

- `MECHANICAL.md` — measured USB daughterboard datum, hole locations, USB overhang, PCB thickness, and extension rule.
- `tools/validate_mechanics.py` — asserts the fixed top/side datum, 1.0 mm thickness, exact NPTH positions/diameters, USB placement, and prevents shortening the daughterboard below the 18 mm reference body.
- `experimental/unni-smartification-c6-10.99-constraints.kicad_pcb` — KiCad 10.99-only copy using native geometric constraints for the rectangular daughterboard outline. Width is driving/fixed at 19 mm; board height is intentionally not fixed so the rear/bottom edge may be extended.

The main `unni-smartification-c6.kicad_pcb` remains KiCad 10.0.5-compatible. The experimental board requires KiCad 10.99 or newer.

## PCB layout (KiCad 10.99)

The manufacturing master `unni-smartification-c6.kicad_pcb` now contains **both physical PCBs in one PCB Editor file**. The USB/power board is the 19 mm wide left island (currently drawn at the full 65 mm available depth; only its 19 mm width/top datum are driving constraints). The ESP/interface board is the 46 x 32 mm right island with width and height constrained. Cross-board electrical connections deliberately remain visible as ratsnest lines until the final JST-PH interconnect pinout is added to the schematic.

