# KiCad / ngspice simulation — Rev A v29

The project is prepared for **KiCad 10.0.5 -> Inspect -> Simulator**. `unni-smartification-c6.wbk` provides four transient views. The simulation harness is part of the same hierarchy but excluded from BOM/PCB, while the ESP sheet is excluded from SPICE and represented by load stimuli.

## Main 10-second scenario

The test intentionally stresses the hardware power arbitration rather than assuming cooperative firmware:

1. **0.0–0.5 s — battery idle:** USB absent, `BOOST_CMD=0`. Expected: SYS follows battery, +3V3 regulated, +5V and UNNI_AC off.
2. **0.5–3.0 s — forced awake from battery:** `BOOST_CMD=3.3 V`. Q3 turns Q2 on; BOOST_IN rises to battery voltage; TPS613222A raises +5V and UNNI_AC.
3. **3.0–5.0 s — USB inserted while BOOST_CMD deliberately stays high:** Q4 must hardware-inhibit Q3, Q2 must open, BOOST_IN/+5V must collapse, while USB continues to supply SYS and UNNI_AC.
4. **5.0–6.0 s — USB on, BOOST_CMD low:** normal USB operation.
5. **6.0–7.0 s — USB removed, BOOST_CMD low:** all 5 V forced-awake rails must shut off while battery/SYS/+3V3 remain valid.
6. **from 7.0 s — second forced-awake cycle:** proves the boost can restart after USB removal.

## Stimuli / loads

- Battery: 3.70 V open-circuit, 120 mΩ series resistance.
- USB: 5 V source with a simulation-only 100 mΩ hot-plug element. When USB is absent the element is 1 GΩ, so VBUS is genuinely open rather than clamped by a 0 V source.
- ESP load: 20 mA baseline with 180 mA radio bursts on +3V3.
- TPS63031 input-power proxy: I2 on SYS, matching the dynamic ESP load at system level.
- TPS613222A: input consumption/discharge is built into its behavioral model; legacy I3 is excluded.
- UNNI_AC validation load: 100 Ω (~47–50 mA).
- Unni battery-side validation load: 330 Ω (~11 mA at 3.7 V).

These are validation stimuli, not measured final product loads.

## Why VBUS may be a few hundred millivolts while unplugged

The SS14 model includes reverse leakage. During internally boosted operation, D3 reverse leakage can weakly raise the unplugged VBUS node. Hardware R13=47 kΩ deliberately clamps this leakage so Q4 stays safely off. In the validated transient VBUS remains below ~0.5 V while USB is absent and forced-awake is active. This is a useful real-world check rather than a simulation artifact we hide.

## Behavioral models

- `AO3400A.lib` — selected detailed user-supplied AO3400 model.
- `AO3400_user_alternatives.lib` — retained alternatives for comparison.
- `AO3401A.lib` — slow/system-level P-MOS model for power-path/high-side switching.
- `MCP73831.lib` — smooth CC/CV + STAT approximation.
- `TPS63031.lib` — 3.3 V system regulator; dynamic input power is represented by I2 in the harness.
- `TPS613222A.lib` — 5 V system model with finite output impedance plus a 56 Ω equivalent input load. The latter corresponds roughly to the current ~50 mA UNNI_AC test load and ensures BOOST_IN discharges promptly when Q2 opens.
- `SS14.lib` — Schottky model including reverse leakage.
- `USB_HOTPLUG.lib` — simulation-only source connection: 100 mΩ when the VPULSE source is high, 1 GΩ when absent.

The regulator models are deliberately not switch-level models; do not use them for ripple, loop stability or EMI.

## Workbook tabs

1. **System power:** +3V3, +5V, +BATT, BOOST_IN, BOOST_CMD, Q2/Q3 gate nodes, SYS, UNNI_AC and VBUS.
2. **3V3 load step:** +3V3, +BATT and SYS around the radio-current bursts.
3. **Charging:** +BATT, CHARGE_STAT and VBUS around USB insertion/removal.
4. **Forced-awake/inhibit:** +5V, BOOST_CMD, UNNI_AC and VBUS.

## Automated direct-from-KiCad test

`tools/validate_kicad.sh` exports the SPICE netlist from the actual KiCad hierarchy, then `tools/make_kicad_transient.py` adds only the analysis/control block. The resulting transient therefore uses the real KiCad connectivity rather than a separately redrawn test circuit.

The validator checks representative points at 0.10, 0.70, 3.10, 6.10 and 7.10 s, including Q2/Q3 gate states and BOOST_IN. See `validation/summary.txt` and `validation/kicad-system.csv`.

## Worst-case battery sweep

`tools/run_worst_case_sweep.py` adds a second, deliberately harsher regression layer. It does **not** redraw the power circuit: every case starts from `spice/exported-from-kicad.cir`, so the sweep exercises the same KiCad connectivity as the normal workbook.

The matrix is:

- battery OCV: **4.2, 3.7, 3.3, 3.0 V**;
- battery internal resistance: **0.1, 0.3, 0.5 ohm**;
- ESP load: 20 mA baseline with a **180 mA radio burst deliberately aligned to the 0.5 s forced-awake startup**;
- UNNI_AC load remains the 100-ohm validation load used by the workbook.

For this stress regression only, the simple SYS current proxy is replaced by a conservative input-power proxy at 88% efficiency. The 5 V boost also gets supplemental low-voltage input loading so the normal 56-ohm behavioral input load does not become unrealistically optimistic as VBAT falls. These modifications live only in the generated sweep decks; the interactive KiCad workbook remains unchanged.

Results are written to `validation/worst-case/worst-case-sweep.csv` and summarized in `validation/worst-case/README.md`. The automated validator fails if +3V3 drops below 3.15 V, the modeled +5 V rail drops below 4.80 V, or UNNI_AC drops below 4.40 V at the steady simultaneous-stress sample.

The 3.0 V OCV cases are intentionally aggressive. A real protected Li-ion cell or protection board may disconnect before the simulated regulator rails fail, so these rows are regulator/power-path stress tests rather than a recommended battery cutoff.

### Running validation outside the author's Linux environment

`tools/run_ngspice_shared.py` is platform-aware. It auto-discovers KiCad/Homebrew/system libngspice on macOS and Linux and falls back to the `ngspice` CLI. Override discovery with `NGSPICE_LIB` or `NGSPICE_BIN` if desired. No `/mnt/data/...` path is required on the user's machine.

## KiCad 10.99 connector simulation note

Physical connectors J1 (USB-C) and J4 (Unni power/battery connector) are explicitly excluded from SPICE. Their connectivity remains present in the electrical netlist, but KiCad no longer emits the unsimulatable placeholder devices `J1 __J1` / `J4 __J4`. This restores direct simulator export after the USB-C symbol was replaced with the Same Sky UJ20 part.
