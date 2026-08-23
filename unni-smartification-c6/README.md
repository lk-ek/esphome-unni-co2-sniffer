# Unni CO2 Smartification — ESP32-C6 Rev A

Target: **KiCad 10.99 nightly or newer**. The current manufacturing master uses a 4-layer stackup and native KiCad 10.99 geometric constraints.

This repository contains two physical PCBs in one KiCad PCB file so cross-board connectivity remains visible while routing. Placement and routing are still work in progress; the schematic and power architecture are substantially defined and covered by reproducible connectivity/SPICE checks.

## Physical boards

- **USB / power board** — 19 mm fixed width, 1.0 mm finished-board target, USB-C on B.Cu at the measured enclosure datum. It contains USB-C/CC, the TPD2E009DBZR USB ESD shunt, the battery-driven 5 V forced-awake boost and the Unni AC/DC interface.
- **ESP / battery board** — maximum 46 x 32 mm. It contains the ESP32-C6-MINI-1, MCP73831 charger and USB/battery load sharing, TPS63031 3.3 V buck-boost, battery/USB/backlight ADCs, battery NTC, CO2/RT/RH taps, touch emulation, and local service controls.

The boards currently use JST-PH connectors where serviceability is useful. If enclosure clearance proves too tight, their exposed through-hole/SMT pads are also suitable for direct wire soldering; this is an assembly fallback, not a separate electrical architecture.

## Power architecture

```text
USB VBUS ──────────────┬─ MCP73831 (≈196 mA, RPROG=5.1k) ── +BATT
                       ├─ Schottky ── SYS ── TPS63031 ── +3V3 ── ESP32-C6
                       └─ Schottky ─────────────────────── UNNI_AC

protected 1S Li-ion ─ +BATT ─ PMOS load share ─────────── SYS
                    └ +BATT ─ boost switch ─ TPS613222A ─ +5V ─ Schottky ─ UNNI_AC
```

Hardware VBUS inhibit prevents the battery 5 V boost from being enabled while real USB power is present. The battery pack used by the design contains its own protection PCB.

## USB

- USB-C sink: independent 5.1 kΩ Rd on CC1 and CC2.
- Native ESP32-C6 USB Full-Speed is routed through 22 Ω series resistors near the ESP.
- USB ESD: **TI TPD2E009DBZR**, a two-line ground-referenced shunt device with no VBUS/VCC rail. This avoids a clamp path that could back-power VBUS and interfere with the hardware boost inhibit.
- `USB_ADC` includes local RC filtering.
- USB differential geometry is intentionally **not final yet**. Route it only after the selected JLCPCB stackup is loaded; final impedance geometry should be checked with JLCPCB's impedance calculator.

## 4-layer JLCPCB-near stackup

The KiCad stackup models JLCPCB's current 4-layer 1.0 mm **JLC3313** construction closely:

| Layer | KiCad model |
| --- | --- |
| F.Cu | 0.035 mm Cu (1 oz external) |
| dielectric 1 | 0.0994 mm 3313 prepreg, εr 4.1 |
| In1.Cu | 0.0152 mm Cu (0.5 oz internal) |
| core | 0.700 mm Nan Ya NP-155F, εr 4.53 |
| In2.Cu | 0.0152 mm Cu (0.5 oz internal) |
| dielectric 3 | 0.0994 mm 3313 prepreg, εr 4.1 |
| B.Cu | 0.035 mm Cu (1 oz external) |

Soldermask is modeled approximately (εr 3.8); JLCPCB's calculator models mask geometry in more detail and remains authoritative for controlled impedance. The manufacturer nominal layer dimensions sum to approximately the requested 1.0 mm construction; finished-board thickness has normal fabrication tolerance.

Both internal layers are intended to remain GND reference planes. Power distribution uses F.Cu/B.Cu pours and stitching vias. High-dv/dt switch nodes remain small local outer-layer copper only. The ESP module antenna has an all-copper-layer keepout.

Current JLCPCB references:
- https://jlcpcb.com/impedance
- https://jlcpcb.com/help/article/user-guide-to-the-jlcpcb-impedance-calculator
- https://jlcpcb.com/capabilities/pcb-capabilities

## Schematic sheets

- `01_usb_power.kicad_sch` — USB-C/CC, TPD2E009DBZR ESD, forced-awake 5 V boost, hardware USB inhibit, Unni AC/DC interface.
- `02_esp_aux.kicad_sch` — protected-battery input, MCP73831 charger, discrete USB/battery load share, TPS63031 3.3 V rail, ESP32-C6, ADC sensing, NTC, passive CO2/RT/RH taps, touch emulation and controls.
- `03_simulation_harness.kicad_sch` — simulation-only stimuli and loads; excluded from BOM/PCB.

## Routing / fabrication baseline

The Board Setup intentionally targets comfortable low-cost JLCPCB geometry rather than absolute fab limits:

- general minimum trace width / copper clearance: 0.15 mm
- routed-edge copper clearance: 0.25 mm
- hole-to-copper baseline: 0.20 mm
- via-hole-to-via-hole baseline: 0.20 mm
- drilled component pad hole-to-hole: 0.45 mm via `.kicad_dru`
- normal preferred via: 0.60/0.30 mm
- dense preset: 0.50/0.20 mm
- zones: 0.25 mm nominal clearance, 0.20 mm hard floor, 0.30 mm thermal gap, 0.40 mm thermal spokes

Preferred netclass widths remain wider than the hard manufacturing minimum. Fine-pitch IC pad escapes may neck down briefly before returning to their preferred class width.

## Simulation and validation

Open `unni-smartification-c6.kicad_pro`, then **Inspect -> Simulator**. `unni-smartification-c6.wbk` contains transient views. See `SIMULATION.md` for model fidelity and test details.

A normal macOS validation run is:

```sh
KICAD_CLI="/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli" ./tools/validate_all.sh
```

If KiCad's ngspice shared library is not auto-discovered, set `NGSPICE_LIB` explicitly. Validation checks:

1. fixed mechanics and both board envelopes;
2. JLCPCB-near stackup and routing-rule presets;
3. schematic load, PDF and KiCad/XML/SPICE netlist export;
4. schematic↔PCB pad/net parity;
5. critical power/USB connectivity, the TPD2E009 pinout, RPROG=5.1 kΩ and USB_ADC filtering;
6. ERC classification;
7. standalone behavioral power regressions;
8. a transient generated directly from KiCad's exported SPICE netlist;
9. a 12-case battery/internal-resistance worst-case sweep.

Behavioral models validate topology, source handover, charger behavior, enable/inhibit states and supply transients. They do **not** predict switch-node ripple, converter control-loop stability, switching loss, RF performance or EMI.

After opening a revision in PCB Editor, **refill all zones (`B`) before trusting DRC**, especially after changes to hole-clearance or stackup rules. KiCad stores filled-zone geometry in the PCB file; cached fills from the previous rule set can otherwise report obsolete clearances until refilled.

## Mechanical reference

See `MECHANICAL.md`. The measured USB-board top datum, hole locations and USB overhang are fixed; the board may extend only toward its rear/bottom direction. The USB-side B.Cu component-clearance window is limited to the first 19 x 18 mm, with an explicit placement keepout below it.


## PCB sheet placement

Both physical PCB islands are kept in the same A4 PCB drawing sheet, but the complete layout is translated into the lower-left half so neither board outline crosses the sheet boundary. The USB/power board top-left datum is currently at `(72.65, 123.2)` mm; the ESP/interface board spans `(110, 125)` to `(156, 157)` mm. This translation is purely a drawing-sheet placement change: all copper, footprints, zones, dimensions, generated via-stitch regions and native 10.99 geometric constraints retain their relative geometry. `tools/validate_mechanics.py` verifies both board envelopes and A4 containment.

## KiCad 10.0.5 compatibility export

The editable project master targets KiCad 10.99 because it uses native geometric PCB constraints and generated via-stitch metadata. Some fabrication/export plugins are only available for stable KiCad 10.0.5, so the repository includes a reproducible compatibility converter instead of downgrading the master.

Create the stable compatibility tree with:

```sh
python3 tools/make_kicad_10_0_5_copy.py
```

The result is written to `dist/kicad-10.0.5/unni-smartification-c6/`. Use that copy for KiCad 10.0.5 plugins and final fabrication exports. Do not make design edits there; regenerate it from the 10.99 master after each relevant change.

For a real KiCad 10.0.5 smoke test (schematic parse/ERC/netlist plus PCB DRC/schematic parity/Gerber/drill export):

```sh
KICAD_10_0_5_CLI=/path/to/kicad-cli tools/validate_kicad_10_0_5_export.sh
```

The converter removes only 10.99 editing/link metadata that stable 10.0.x cannot safely consume: native geometric constraint objects, generated via-stitch descriptors and design-block `lib_id` links attached to instantiated groups. The group members themselves remain local in the schematic/PCB, already-instantiated vias remain explicit PCB vias, footprint placements are translated to the 10.0.x representation, and the mechanical geometry continues to be guarded by `tools/validate_mechanics.py` in the master project.

## Per-board JLCPCB Gerber export

The design intentionally keeps both physical boards in one `.kicad_pcb`, while JLCPCB fabrication needs one closed board outline per order. Generate separate fabrication packages with:

```sh
KICAD_10_0_X_CLI=/path/to/KiCad-10.0.x/kicad-cli \
  python3 tools/export_jlc_gerbers.py
```

The script first creates a temporary stable-KiCad compatibility copy, identifies the 19 mm USB/power island and the 46 x 32 mm ESP/interface island from `Edge.Cuts`, then creates isolated temporary boards and exports each with KiCad 10.0.x. Outputs are written to `dist/jlc/`:

- `unni-usb-power-jlc-gerbers.zip`
- `unni-esp-interface-jlc-gerbers.zip`

Each ZIP contains the four copper layers, front/back solder mask, front/back silkscreen, `Edge.Cuts`, Gerber job file and separate PTH/NPTH Excellon drill files. Paste layers are intentionally omitted from the bare-PCB fabrication ZIPs. Regenerate the packages after every PCB change.
