#!/usr/bin/env python3
"""Validate fixed Unni replacement-PCB mechanical datum.

The top/left/right datum, USB placement and four mechanical holes are fixed.
The bottom edge is intentionally allowed to move only downward from the 18 mm
reference body height.
"""
from __future__ import annotations

from pathlib import Path
import math
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
PCB = ROOT / "unni-smartification-c6.kicad_pcb"
S = PCB.read_text(errors="strict")

TOL = 1e-6
LEFT = 222.65
TOP = 78.2
RIGHT = 241.65
REF_BOTTOM = 96.2


def fail(msg: str) -> None:
    print(f"MECHANICAL ERROR: {msg}", file=sys.stderr)
    raise SystemExit(6)


def close(a: float, b: float, tol: float = TOL) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)


# Board thickness.
m = re.search(r"\(general\s+\(thickness\s+([0-9.]+)\)", S)
if not m:
    fail("board thickness not found")
thickness = float(m.group(1))
if not close(thickness, 1.0):
    fail(f"board thickness is {thickness:g} mm, expected 1.0 mm")

# Manufactured Edge.Cuts contain two physical PCB islands in the same file:
# the mechanically fixed USB/power board and the ESP/interface board.
lines = []
for m in re.finditer(
    r"\(gr_line\s+\(start\s+([-0-9.]+)\s+([-0-9.]+)\)\s+"
    r"\(end\s+([-0-9.]+)\s+([-0-9.]+)\).*?\(layer\s+\"Edge\.Cuts\"\)",
    S,
    re.S,
):
    lines.append(tuple(float(x) for x in m.groups()))


def rectangle_from_lines(group: list[tuple[float, float, float, float]], name: str):
    if len(group) != 4:
        fail(f"{name}: expected four Edge.Cuts lines, found {len(group)}")
    xs = [v for line in group for v in (line[0], line[2])]
    ys = [v for line in group for v in (line[1], line[3])]
    x1, x2, y1, y2 = min(xs), max(xs), min(ys), max(ys)
    expected = {
        (x1, y1, x2, y1), (x2, y1, x2, y2),
        (x2, y2, x1, y2), (x1, y2, x1, y1),
    }
    if set(group) != expected:
        fail(f"{name}: Edge.Cuts outline is not a closed axis-aligned rectangle")
    return x1, y1, x2, y2

usb_lines = [ln for ln in lines if max(ln[0], ln[2]) < 250.0]
esp_lines = [ln for ln in lines if min(ln[0], ln[2]) > 250.0]
if len(lines) != 8:
    fail(f"expected 8 Edge.Cuts lines for two PCB islands, found {len(lines)}")

x1, y1, x2, y2 = rectangle_from_lines(usb_lines, "USB board")
if not (close(x1, LEFT) and close(y1, TOP) and close(x2, RIGHT)):
    fail(f"USB fixed top/side datum changed: got {(x1, y1, x2)}, expected {(LEFT, TOP, RIGHT)}")
if y2 + TOL < REF_BOTTOM:
    fail(f"USB bottom edge moved upward to y={y2:g}; minimum/reference y is {REF_BOTTOM:g}")
if not close(x2 - x1, 19.0):
    fail(f"USB board width is {x2-x1:g} mm, expected 19.0 mm")

ex1, ey1, ex2, ey2 = rectangle_from_lines(esp_lines, "ESP board")
if not (close(ex2-ex1, 46.0) and close(ey2-ey1, 32.0)):
    fail(f"ESP board envelope is {ex2-ex1:g} x {ey2-ey1:g} mm, expected 46 x 32 mm")

# Confirm driving fixed-length constraints for the two-board study are present.
fixed_vals = [float(v) for v in re.findall(r"\(constraint\s+\(type\s+fixed_length\).*?\(value\s+([-0-9.]+)\)", S, re.S)]
for needed in (19.0, 46.0, 32.0):
    if not any(close(v, needed) for v in fixed_vals):
        fail(f"missing fixed_length geometric constraint for {needed:g} mm")

# Footprint parser: locate each board-only mechanical footprint by Reference and
# inspect its top-level placement and NPTH drill.
def footprint_for_reference(ref: str) -> str:
    # Balanced scan from each top-level footprint opening.
    for m0 in re.finditer(r"\(footprint\s+", S):
        start = m0.start()
        depth = 0
        in_str = False
        esc = False
        end = None
        for i in range(start, len(S)):
            c = S[i]
            if in_str:
                if esc:
                    esc = False
                elif c == "\\":
                    esc = True
                elif c == '"':
                    in_str = False
                continue
            if c == '"':
                in_str = True
            elif c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        if end is None:
            fail("unterminated footprint block")
        block = S[start:end]
        if re.search(rf'\(property\s+"Reference"\s+"{re.escape(ref)}"', block):
            return block
    fail(f"footprint {ref} not found")
    raise AssertionError


def check_hole(ref: str, x: float, y: float, drill: float) -> None:
    b = footprint_for_reference(ref)
    # KiCad 10 uses top-level (at ...); 10.99 uses transform/translate.
    m = re.search(r"\(transform\s+\(translate\s+([-0-9.]+)\s+([-0-9.]+)\)", b, re.S)
    if not m:
        m = re.search(r"\n\t\t\(at\s+([-0-9.]+)\s+([-0-9.]+)(?:\s+[-0-9.]+)?\)", b)
    if not m:
        fail(f"{ref} placement not found")
    gotx, goty = map(float, m.groups())
    if not (close(gotx, x) and close(goty, y)):
        fail(f"{ref} center is ({gotx:g}, {goty:g}), expected ({x:g}, {y:g})")
    dm = re.search(r"\(pad\s+\"\"\s+np_thru_hole\s+circle.*?\(drill\s+([0-9.]+)\)", b, re.S)
    if not dm:
        fail(f"{ref} NPTH drill not found")
    gotd = float(dm.group(1))
    if not close(gotd, drill):
        fail(f"{ref} drill is {gotd:g} mm, expected {drill:g} mm")
    if "board_only" not in b:
        fail(f"{ref} must remain board_only")


check_hole("H1", RIGHT - 2.0, TOP + 9.0, 2.0)
check_hole("H2", RIGHT - 1.5, TOP + 13.0, 1.0)
check_hole("H3", RIGHT - 9.5, TOP + 13.7, 2.0)
check_hole("H4", LEFT + 2.5, TOP + 13.5, 1.5)

# USB/NPTH measurements were taken while looking at the enclosure/B.Cu side.
# PCBNew's normal F.Cu view mirrors X, so the measured shield span 6..15 mm
# becomes 4..13 mm (center X=8.5 mm) in board coordinates.  J1 is oriented
# with the receptacle mouth outward and positioned so the metal shield body,
# rather than the contact/pad end, protrudes about 1.0 mm past the top edge.
j1 = footprint_for_reference("J1")
m = re.search(
    r"\(transform\s+\(translate\s+([-0-9.]+)\s+([-0-9.]+)\)\s+\(rotate\s+([-0-9.]+)\)",
    j1, re.S,
)
if not m:
    m = re.search(r"\n\t\t\(at\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)\)", j1)
if not m:
    fail("J1 placement not found")
jx, jy, ja = map(float, m.groups())
expected_jx = LEFT + 8.5
# The Same Sky UJ20 footprint origin sits at the receptacle mouth.  The measured
# metal shield protrudes ~1 mm beyond the top datum; allow normal footprint/grid
# rounding rather than tying the validator to the previous connector footprint.
expected_jy = TOP - 1.0
if not (close(jx, expected_jx, 0.05) and close(jy, expected_jy, 0.10) and close((ja % 360), 0.0)):
    fail(
        f"J1 placement is ({jx:g}, {jy:g}, {ja:g} deg), expected "
        f"({expected_jx:g}, {expected_jy:g}, 0 deg)"
    )

# Component-side mechanics. The housing clearance on the USB board exists only
# on B.Cu in the first 19 x 18 mm datum rectangle.
def fp_layer(block: str) -> str:
    m = re.search(r'\(layer\s+"([FB]\.Cu)"\)', block)
    if not m:
        fail("footprint copper side not found")
    return m.group(1)

if fp_layer(j1) != "B.Cu":
    fail("J1 USB-C receptacle must be on B.Cu / enclosure side")
for ref in ("R1", "R2"):
    b = footprint_for_reference(ref)
    if fp_layer(b) != "B.Cu":
        fail(f"{ref} USB-C CC resistor must remain on B.Cu")
    pm = re.search(r"\(transform\s+\(translate\s+([-0-9.]+)\s+([-0-9.]+)\)", b, re.S)
    if not pm:
        fail(f"{ref} placement not found")
    px, py = map(float, pm.groups())
    if not (LEFT <= px <= RIGHT and TOP <= py <= REF_BOTTOM):
        fail(f"{ref} is outside the 19 x 18 mm B-side clearance window")

# Explicit placement keepout protects the flush B-side region below the datum window.
if 'B-side component keepout beyond 19x18 mm clearance' not in S or '(footprints not_allowed)' not in S:
    fail("missing B-side footprint placement keepout below the 19 x 18 mm clearance window")

# All JSTs are SMT side-entry PH connectors, not THT/XH, and remain on F.Cu.
for ref in ("J2", "J4", "J10", "J20", "J21", "J22"):
    b = footprint_for_reference(ref)
    if 'JST_PH_S' not in b or '-PH-SM4-TB_' not in b or '(attr smd)' not in b:
        fail(f"{ref} is not an SMT side-entry JST-PH SxB-PH-SM4-TB footprint")
    if fp_layer(b) != "F.Cu":
        fail(f"{ref} must be on F.Cu")

# Hand-solder variants for all 0603/0805 capacitors currently used in the design.
for mref in re.finditer(r'\(property\s+"Reference"\s+"(C[0-9]+)"', S):
    ref = mref.group(1)
    b = footprint_for_reference(ref)
    if ('C_0603_1608Metric' in b or 'C_0805_2012Metric' in b) and '_HandSolder' not in b:
        fail(f"{ref} does not use a hand-solder capacitor footprint")

# RT1 is intentionally leaded/THT on the ESP board to make assembly and sensor placement easier.
rt1 = footprint_for_reference("RT1")
if 'Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal' not in rt1 or '(attr through_hole)' not in rt1:
    fail("RT1 must use the selected leaded THT footprint")

print(
    "Mechanical assertions passed: 1.0 mm PCB; USB board 19 mm fixed width/top datum with 4 NPTH holes; "
    "USB/Rcc on B.Cu within the 19 x 18 mm clearance; B-side keepout below it; "
    "SMT side-entry JST-PH connectors; hand-solder capacitors; THT RT1; ESP board envelope 46 x 32 mm."
)
