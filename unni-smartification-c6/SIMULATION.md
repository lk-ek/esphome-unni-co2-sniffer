# KiCad / ngspice simulation

The project is prepared for KiCad 10.0.5 **Inspect → Simulator**. `unni-smartification-c6.wbk` contains prepared transient views. The simulation harness is part of the same hierarchical schematic, but is excluded from BOM/PCB.

## KiCad simulation harness

`03_simulation_harness.kicad_sch` supplies system-level test sources and loads while `02_esp_aux.kicad_sch` is excluded from SPICE as a whole. The ESP32-C6 is represented by load stimuli rather than a nonexistent transistor-level MCU model.

### Stimulus

- USB source: 0 → 5 V hot-plug at 3 s, including 100 mΩ source/cable resistance.
- Battery: 3.70 V open-circuit plus 120 mΩ internal resistance.
- `BOOST_CMD`: asserted before USB insertion and deliberately kept asserted across hot-plug. This tests the hardware USB-inhibit path rather than relying on firmware cooperation.
- 3V3 load: 20 mA baseline with 180 mA radio bursts.
- UNNI_AC test load: 100 Ω.
- Unni battery-side test load: 330 Ω.

These load values are validation stimuli, not claims about measured device consumption.

## Charger

The final hardware and KiCad simulation both use autonomous MCP73831 operation. `PROG` is returned to ground through 3.3 kΩ; there is no MCU-controlled charge-enable switch. The behavioral MCP73831 model implements a smooth system-level CC/CV approximation and STAT behavior suitable for power-path tests.

## Models

- `AO3400A.lib` — detailed selected user-supplied model.
- `AO3400_user_alternatives.lib` — all three supplied variants, retained for comparison testing.
- `AO3401A.lib` — system-level P-MOS model for load-sharing/high-side-switch behavior.
- `MCP73831.lib` — smooth behavioral CC/CV charger model.
- `TPS63031.lib` — fixed-3.3 V behavioral buck-boost model.
- `TPS613222A.lib` — fixed-5 V behavioral boost model.
- `SS14.lib` — Schottky diode subcircuit.

The converter models intentionally omit switching-node waveform, loop dynamics, ripple, and EMI. The optional external TPS613222A Schottky is excluded from SPICE because a static behavioral boost model has no physical switching waveform for that diode to rectify.

## Standalone regression decks

`spice/sim/` contains:

- `ao3400_compare.cir`
- `mcp73831_charge.cir`
- `3v3_loadstep.cir`
- `system_power.cir`

All are exercised by `tools/validate_kicad.sh` using KiCad's bundled `libngspice`.

## Direct KiCad-netlist validation

The validator exports `spice/exported-from-kicad.cir` from the actual hierarchy, adds only a transient analysis command, then runs that generated deck through ngspice. This prevents a separate hand-written simulation schematic from masking wiring mistakes in the KiCad source.

Validated operating points include:

- around 1 s, battery operation with forced-awake active: 3V3 remains regulated and `UNNI_AC` is raised to the 5 V domain through the boost/SS14 path;
- around 4 s, external USB is present while `BOOST_CMD` is still asserted: USB supplies the system and `UNNI_AC`, exercising the hardware inhibit topology.

The exact generated samples from the most recent run are printed by `tools/validate_kicad.sh` and the waveform data are saved in `validation/kicad-system.csv`.
