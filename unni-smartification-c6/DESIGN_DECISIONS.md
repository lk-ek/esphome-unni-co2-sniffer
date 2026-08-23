# Design decisions — Rev A v29 validated

- The original Unni main-board traces remain untouched. CO2, RT/RH, touch and backlight connections are T-taps so the Unni remains usable without the ESP board.
- The original small USB daughterboard is replaced. This separates raw external USB `VBUS` from the internally generated `UNNI_AC` rail and provides native USB D+/D- to the ESP32-C6.
- USB-C CC1/CC2 use 5.1 kΩ Rd resistors.
- Battery remains directly connected to `UNNI_DC`.
- Charger is MCP73831T-2ACI/OT with 3.3 kΩ PROG (~300 mA nominal), operating autonomously whenever VBUS is present. This avoids a flat-battery deadlock where an unpowered ESP would have to enable charging.
- USB and battery feed `SYS` through SS14 + AO3401A load sharing. D1 orientation is Anode=VBUS, Cathode=SYS, preventing SYS-to-VBUS backfeed.
- `SYS` feeds the TPS63031 fixed 3.3 V buck-boost. FB is tied directly to VOUT/+3V3 as required for the fixed-output version.
- Forced-awake 5 V uses TPS613222A. Q2 (AO3401A) disconnects the boost input from the battery when not required.
- Q3 (AO3400A) pulls Q2 gate low when `BOOST_CMD` is asserted. Q4 (AO3400A) pulls the Q3 gate low whenever VBUS is present, so external USB hardware-inhibits the internal boost even if firmware leaves `BOOST_CMD` high.
- R13 is 47 kΩ from VBUS to GND. This intentionally provides a firm VBUS pull-down because the two SS14 paths have reverse leakage; without it, internally generated `UNNI_AC` can raise the otherwise-unplugged VBUS node enough to partially bias Q4. The 47 kΩ costs only ~106 µA when USB is actually present.
- `UNNI_AC` is sourced through separate SS14 paths from external VBUS and internal +5V. This is simple, cheap and prevents deliberate forward backfeed between sources.
- Battery, USB and battery-temperature ADC filters are explicitly wired. Optional DNP capacitors are connected to their intended nets rather than left electrically floating.
- GPIO4/5/8/9/15 are ESP32-C6 strapping pins; normal runtime Unni signals avoid them. GPIO9 remains BOOT-only.
- Simulation-only circuitry lives on `03_simulation_harness.kicad_sch` and is excluded from BOM/PCB.

## Mechanical datum

- The replacement USB daughterboard uses the top PCB edge as the authoritative mechanical datum; USB points outward/upward.
- Reference body size is 19.0 mm wide x 18.0 mm high, with 1.0 mm PCB thickness. The board may grow only toward the rear/bottom; all measured hole Y offsets and USB placement remain referenced to the top edge.
- The four measurements were taken from the B.Cu/enclosure-side view: (2.0, 9.0) Ø2.0, (1.5, 13.0) Ø1.0, (9.5, 13.7) Ø2.0, and (16.5, 13.5) Ø1.5 mm. In PCBNew/F.Cu coordinates their X positions are mirrored to 17.0, 17.5, 9.5 and 2.5 mm respectively.
- USB shield nominal span measured from B.Cu is X=6.0..15.0 mm; in PCBNew/F.Cu coordinates it is X=4.0..13.0 mm, center X=8.5 mm. The metal shield mouth, not the contact/pad end, protrudes 1.0 mm beyond the top PCB edge.
- KiCad 10.99 geometric constraints are kept in an experimental board copy because 10.0.5 cannot load the 10.99 constraint file format and the current solver does not robustly constrain whole mounting-hole footprints as rigid units.

## PCB routing / fabrication baseline (2026-08-21)

The project targets inexpensive JLCPCB 1 oz fabrication without routing at the fab's absolute limits. Board Setup therefore uses a conservative 0.15 mm global minimum trace/clearance baseline, 0.30 mm copper-to-edge clearance, 0.30 mm hole-to-copper clearance, and a preferred general-purpose 0.60/0.30 mm via. A 0.50/0.20 mm via is available as a dense-routing preset, not the normal default.

Predefined routing widths are 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.80, 1.00 and 1.20 mm. Net classes select useful defaults from these presets: Signal 0.20 mm, Sensitive 0.20 mm with 0.25 mm clearance, USB 0.20/0.20 mm width/gap, +3V3 0.40 mm, VBUS/+5V/UNNI_AC 0.50 mm, battery/SYS/BOOST_IN 0.80 mm, switch nodes 0.80 mm with extra clearance, and GND tracks 0.50 mm. USB width/gap is a practical routing default only; if controlled impedance is ordered, it must be recalculated for the selected JLCPCB stackup.

Copper zones default to 0.25 mm clearance, 0.20 mm minimum copper thickness, 0.30 mm thermal gap, 0.40 mm thermal spokes and 1 mm² minimum island area. Existing copper zones are normalized to these values. `unni-smartification-c6.kicad_dru` adds DRC guards for zone clearance and per-netclass minimum/preferred widths.

## Rules / net-label cleanup (2026-08-23)

The PCB rules were simplified after the power stages moved onto the ESP board.  Netclass trace widths remain the preferred interactive-router defaults, but hard minimum-width rules for 3V3, 5V, battery/SYS, switching and GND were removed because they incorrectly rejected the short neck-downs required at fine-pitch IC pads.  Switching and Battery_SYS clearances are 0.20 mm so the TPS63031 package itself is legal; zone geometry still defaults to 0.25 mm clearance, with a 0.20 mm hard DRC floor to avoid 0.2499 mm rounding false positives.  The general copper-edge hard minimum is 0.25 mm.

Physical wire-pad aliases that were electrically identical to existing nets were removed: CONN_GND, CONN_VBUS, CONN_USB_P, CONN_USB_N and CONN_+BATT now use GND, VBUS, USB_RAW_P, USB_RAW_N and +BATT directly.  Connector-side USB nets before U4 remain deliberately named USBPort_P/USBPort_N; the protected side remains USB_RAW_P/USB_RAW_N.

The ESP sheet is now included in SPICE because MCP73831 and TPS63031 live there.  Physical solder-wire pads and the ESP32-C6 module are explicitly excluded from simulation, preserving the complete power-stage simulation without placeholder devices.
