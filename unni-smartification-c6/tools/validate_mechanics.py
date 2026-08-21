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

# Exactly one manufactured outer Edge.Cuts rectangle is expected at this stub stage.
rects = []
for m in re.finditer(
    r"\(gr_rect\s+\(start\s+([-0-9.]+)\s+([-0-9.]+)\)\s+"
    r"\(end\s+([-0-9.]+)\s+([-0-9.]+)\).*?\(layer\s+\"Edge\.Cuts\"\)",
    S,
    re.S,
):
    rects.append(tuple(float(x) for x in m.groups()))
if len(rects) != 1:
    fail(f"expected one Edge.Cuts rectangle, found {len(rects)}")
x1, y1, x2, y2 = rects[0]
if not (close(x1, LEFT) and close(y1, TOP) and close(x2, RIGHT)):
    fail(f"fixed top/side board datum changed: got {(x1, y1, x2)}, expected {(LEFT, TOP, RIGHT)}")
if y2 + TOL < REF_BOTTOM:
    fail(f"bottom edge moved upward to y={y2:g}; minimum/reference y is {REF_BOTTOM:g}")
if not close(x2 - x1, 19.0):
    fail(f"board width is {x2-x1:g} mm, expected 19.0 mm")

# Footprint parser: locate each board-only mechanical footprint by Reference and
# inspect its top-level placement and NPTH drill.
def footprint_for_reference(ref: str) -> str:
    # Board footprint blocks are balanced S-expressions; scan from each footprint start.
    for start in (m.start() for m in re.finditer(r"\n\t\(footprint\s+", S)):
        depth = 0
        in_str = False
        esc = False
        end = None
        for i in range(start + 1, len(S)):
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
    # The first (at ...) in a board footprint is its placement.
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


check_hole("H1", LEFT + 2.0, TOP + 9.0, 2.0)
check_hole("H2", LEFT + 1.5, TOP + 13.0, 1.0)
check_hole("H3", RIGHT - 9.5, TOP + 13.7, 2.0)
check_hole("H4", RIGHT - 2.5, TOP + 13.5, 1.5)

# USB footprint: exact center x from measured 6/4 mm shield clearances.  The GCT
# library footprint's "PCB Edge" marker is local y=+3.675 mm.  At 180 degrees,
# y_origin = TOP + 3.675 puts that marker exactly on the top datum.
j1 = footprint_for_reference("J1")
m = re.search(r"\n\t\t\(at\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)\)", j1)
if not m:
    fail("J1 placement not found")
jx, jy, ja = map(float, m.groups())
expected_jx = LEFT + 10.5
expected_jy = TOP + 3.675
if not (close(jx, expected_jx) and close(jy, expected_jy) and close((ja % 360), 180.0)):
    fail(
        f"J1 placement is ({jx:g}, {jy:g}, {ja:g} deg), expected "
        f"({expected_jx:g}, {expected_jy:g}, 180 deg)"
    )

print(
    "Mechanical assertions passed: 1.0 mm PCB; 19 mm fixed width/top datum; "
    "4 NPTH holes; USB centered/aligned; bottom may extend downward."
)
