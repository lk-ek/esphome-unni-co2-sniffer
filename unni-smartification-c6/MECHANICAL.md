# Mechanical constraints

## USB daughterboard datum

The measured USB daughterboard uses the **top PCB edge** as the mechanical datum. The USB receptacle points upward/outward. The physical measurements were taken from the **back / B.Cu view**. The table below preserves those measured coordinates: top-left of the B.Cu view is `(0, 0)`, +X to the right and +Y toward the rear/bottom. PCBNew's normal F.Cu view therefore mirrors X across the 19 mm width.

The measured reference board is **19.0 mm wide x 18.0 mm PCB-only height** and **1.0 mm thick**. The board may be extended toward +Y if more area is needed; the top datum, USB position, and all hole Y coordinates remain fixed. With the measured USB receptacle overhang, the reference assembly is 19.0 mm high including the 1.0 mm outward overhang beyond the top PCB edge.

| Feature | X from left | Y from top | Diameter / extent |
| --- | ---: | ---: | ---: |
| Left mounting hole | 2.0 mm | 9.0 mm | 2.0 mm NPTH |
| Small left hole | 1.5 mm | 13.0 mm | 1.0 mm NPTH |
| Center mounting hole | 9.5 mm | 13.7 mm | 2.0 mm NPTH |
| Right hole | 16.5 mm | 13.5 mm | 1.5 mm NPTH |
| USB shield nominal left edge | 6.0 mm | top edge datum | — |
| USB shield nominal right edge | 15.0 mm | top edge datum | — |
| USB outward overhang | — | -1.0 mm | assembly extent |

The USB shield dimensions imply a nominal 9.0 mm shield width and a measured B.Cu-view center at X = 10.5 mm. In PCBNew/F.Cu board coordinates this is mirrored to X = 8.5 mm (nominal shield span 4.0..13.0 mm). The connector is oriented so the metal shield mouth protrudes 1.0 mm beyond the top PCB edge; the contact/pad end remains over the PCB.

## KiCad 10.99 geometric constraints

The mechanical values above are authoritative even where they are not yet represented by solver constraints. In the tested KiCad 10.99 nightly, geometric constraints currently work primarily on drawing primitives; constraining an entire mounting-hole footprint as a rigid unit is still an open usability limitation. Therefore the holes are implemented as board-only NPTH footprints at exact datum coordinates, while the board outline and dimensions remain suitable for later solver-driven construction geometry.


## Component-side clearance (2026-08-21)

- The measured 19 x 18 mm mechanical clearance applies to the **back / B.Cu side** of the USB board.
- The USB-C receptacle is mounted on **B.Cu** and still protrudes 1 mm beyond the top PCB edge.
- The two USB-C CC 5.1 kOhm resistors (R1/R2) may also be on B.Cu inside that 19 x 18 mm window.
- Below y = 18 mm from the USB-board top datum, the back side lies flat against the enclosure; therefore **no B-side component bodies are allowed there**. The PCB contains an explicit B-side footprint placement keepout for this region.
- All other USB/power-board components are placed on F.Cu.
- JST connectors use SMT, side-entry JST-PH footprints (SxB-PH-SM4-TB family); through-hole JST footprints are not permitted because the rear side must remain flush.


## Assembly-friendly footprints (2026-08-21)

- Use official KiCad `*_HandSolder` footprints wherever a compatible passive footprint exists.
- 0603 capacitors: `C_0603_1608Metric_Pad1.08x0.95mm_HandSolder`.
- 0805 capacitors: `C_0805_2012Metric_Pad1.18x1.45mm_HandSolder`.
- 0805 resistors already use the KiCad hand-solder footprint.
- RT1 is intentionally THT on the ESP board. Until an exact NTC MPN is selected, the mechanically forgiving `R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal` footprint is used for a small leaded/bead NTC with bent leads. Replace it with an MPN-specific footprint once the thermistor is selected.
- All JST-PH connectors are SMT side-entry SxB-PH-SM4-TB footprints. Their cable mouths face toward free board area rather than directly out over an edge.
