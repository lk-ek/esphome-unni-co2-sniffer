# Unni CO2 Smartification — ESP32-C6 Rev A v28 final

Target: **KiCad 10.0.5**.

This is the final schematic + simulation project for the current Rev-A architecture. PCB files are intentionally not included; PCB generation was explicitly dropped in favor of finishing and validating the schematic first.

## Sheets

- `01_usb_power.kicad_sch` — replacement USB-C interface, MCP73831 charger, discrete USB/battery load sharing, TPS63031 3.3 V rail, TPS613222A forced-awake 5 V rail, hardware USB inhibit, and Unni AC/DC outputs.
- `02_esp_aux.kicad_sch` — ESP32-C6-MINI-1, native USB differential pair, ADC sensing, battery NTC, passive CO2/RT/RH taps, touch emulation, and backlight sensing.
- `03_simulation_harness.kicad_sch` — simulation-only sources and loads; excluded from BOM/PCB.

## Charging

The charger is **MCP73831T-2ACI/OT** with `RPROG = 3.3 kΩ` (~300 mA nominal). It runs autonomously whenever VBUS is present. Charging is deliberately **not** gated by the ESP, avoiding a flat-battery deadlock where the MCU cannot boot to enable charging. `STAT`, battery voltage, and battery NTC remain observable by the ESP.

## Power architecture

- USB VBUS and battery are ORed onto `SYS` with SS14 + AO3401A load sharing.
- `SYS` feeds the TPS63031 fixed-3.3 V buck-boost for the ESP32-C6.
- Battery feeds Unni `DC` directly.
- Battery can feed the TPS613222A 5 V boost through an AO3401A high-side switch.
- AO3400A logic implements the forced-awake command and **hardware USB inhibit**, so the internal boost cannot remain enabled when external USB is present.
- `UNNI_AC` is sourced from USB or the 5 V boost through separate SS14 paths, preventing backfeed.

## USB

The replacement USB-C daughterboard uses proper 5.1 kΩ Rd resistors on CC1/CC2. D+/D- are routed to the ESP32-C6 native USB pins through 22 Ω series resistors. The two sides of the resistors are explicitly named as differential pairs:

- connector side: `USB_RAW_P` / `USB_RAW_N`
- MCU side: `USB_P` / `USB_N`

## Simulation directly in KiCad

Open `unni-smartification-c6.kicad_pro`, then **Inspect → Simulator**. The workbook `unni-smartification-c6.wbk` provides prepared transient views. See `SIMULATION.md` for model fidelity and stimulus details.

## Validation

`tools/validate_kicad.sh` performs a reproducible validation against KiCad 10.0.5:

1. load and PDF-export the full hierarchy;
2. export both KiCad XML and SPICE netlists;
3. run ERC;
4. assert 30 critical pin/net relationships plus the autonomous MCP73831 PROG return;
5. run four standalone ngspice behavioral tests;
6. generate and run a transient **from the SPICE netlist exported by KiCad itself**;
7. sanity-check battery-only forced-awake and USB-hotplug operating points.

Run with:

```sh
KICAD_CLI=/path/to/kicad-cli ./tools/validate_kicad.sh
```

In the extracted Linux AppImage used for this project, ERC reports only `lib_symbol_issues` and `footprint_link_issues` because that headless environment lacks a normal desktop user's global library tables. No electrical ERC categories are present. See `validation/summary.txt` and `validation/erc.rpt`.

## Model limitations

AO3400A uses the selected detailed user-supplied model. AO3401A and the power ICs are system-level behavioral models. They validate topology, source handover, enable/inhibit behavior, charger CC/CV behavior, and supply transients. They are not suitable for switching ripple, control-loop stability, or EMI analysis.
