# Mechanical constraints

## USB daughterboard datum

The measured USB daughterboard uses the **top PCB edge** as the mechanical datum. The USB receptacle points upward/outward. Coordinates below use the top-left PCB corner as `(0, 0)`, +X to the right and +Y toward the rear/bottom of the board.

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

The USB shield dimensions imply a nominal 9.0 mm shield width and a nominal center at X = 10.5 mm. The GCT USB4105 KiCad footprint body is 8.94 mm wide, so its body edges differ from the measured nominal shield edges by about 0.03 mm per side when centered on X = 10.5 mm.

## KiCad 10.99 geometric constraints

The mechanical values above are authoritative even where they are not yet represented by solver constraints. In the tested KiCad 10.99 nightly, geometric constraints currently work primarily on drawing primitives; constraining an entire mounting-hole footprint as a rigid unit is still an open usability limitation. Therefore the holes are implemented as board-only NPTH footprints at exact datum coordinates, while the board outline and dimensions remain suitable for later solver-driven construction geometry.
