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


## Component-side clearance (current Rev A)

- The measured 19 x 18 mm mechanical clearance applies to the **back / B.Cu side** of the USB board.
- The USB-C receptacle is mounted on **B.Cu** and protrudes about 1 mm beyond the top PCB edge.
- The two USB-C CC 5.1 kΩ resistors may remain on B.Cu inside the 19 x 18 mm clearance window.
- Below y = 18 mm from the USB-board top datum, the back side lies flat against the enclosure; **no B-side component bodies are allowed there**. The PCB contains an explicit B-side footprint-placement keepout.
- Other USB/power-board components are placed on F.Cu unless explicitly validated inside the B-side clearance window.
- J4 on the USB/power board is an SMT side-entry JST-PH connector; no through-hole signal connector is allowed through the flush portion of that board.
- The ESP board may use THT components and currently uses vertical JST-PH connectors for battery/inter-board/sensor connections. If connector body clearance is insufficient in the enclosure, direct wire soldering to those connector pads is an accepted assembly fallback.

## Assembly-friendly footprints

- Use `*_HandSolder` footprints wherever a compatible passive footprint exists.
- 0603 capacitors: `C_0603_1608Metric_Pad1.08x0.95mm_HandSolder`.
- 0805 capacitors: `C_0805_2012Metric_Pad1.18x1.45mm_HandSolder`.
- 0805 resistors use hand-solder footprints where available.
- L1: project-local Murata LQH3NPN 3.0 x 3.0 mm hand-solder footprint.
- L2: project-local Murata DFE322520FD 3.2 x 2.5 mm hand-solder footprint.
- SW1/SW2: project-local TVAF36-N014JW-R hand-solder footprint.
- RT1 is a removable external NTC on a 2-pin JST-PH THT connector.

## RF / four-layer keepout

The ESP32-C6 module antenna region must remain free of copper on **F.Cu, In1.Cu, In2.Cu and B.Cu**. The surrounding enclosure region is plastic; do not route cables or add metal hardware through the antenna near field during final assembly.
