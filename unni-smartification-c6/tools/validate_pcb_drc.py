#!/usr/bin/env python3
"""Classify PCB DRC output for the intentional two-island master board.

The combined master keeps both physical PCBs in one KiCad board so the inter-board
ratsnest remains visible. Therefore exactly six cross-board nets are expected to be
reported as unconnected by PCBNew. All other DRC errors are prototype blockers.
Known warning categories are pinned to the reviewed baseline. New categories or
count increases are fatal, and dangling vias are always prototype blockers.
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path
import re
import sys

REPORT = Path(sys.argv[1] if len(sys.argv) > 1 else "validation/pcb-drc.rpt")
text = REPORT.read_text(errors="replace")

EXPECTED_INTERBOARD = {"VBUS", "GND", "+BATT", "USB_RAW_N", "USB_RAW_P", "BOOST_CMD"}
WARNING_BASELINE = {
    "silk_overlap": 33,
    "lib_footprint_mismatch": 18,
    "text_height": 6,
    "lib_footprint_issues": 5,
    "silk_over_copper": 4,
    "silk_edge_clearance": 2,
    "nonmirrored_text_on_back_layer": 2,
}
FATAL_WARNINGS = {"via_dangling"}

# A violation starts at a category header and extends to the next category/header footer.
blocks = re.split(r"(?=^\[[^\]]+\]:)", text, flags=re.M)
errors: list[tuple[str, str]] = []
warnings: list[str] = []
for block in blocks:
    m = re.match(r"^\[([^\]]+)\]:", block)
    if not m:
        continue
    category = m.group(1)
    if '; error' in block:
        errors.append((category, block))
    elif '; warning' in block:
        warnings.append(category)

seen_interboard: set[str] = set()
unexpected: list[tuple[str, str]] = []
for category, block in errors:
    if category != "unconnected_items":
        unexpected.append((category, block))
        continue
    # Net names occur in square brackets on the item lines. Ignore the category header.
    names = set(re.findall(r"\[([^\]]+)\]", "\n".join(block.splitlines()[1:])))
    names &= EXPECTED_INTERBOARD
    if len(names) != 1:
        unexpected.append((category, block))
        continue
    seen_interboard |= names

missing = EXPECTED_INTERBOARD - seen_interboard
extra_count = len([1 for c, _ in errors if c == "unconnected_items"]) - len(seen_interboard)
if missing or extra_count:
    unexpected.append(("unconnected_items", f"expected exactly one cross-board disconnect for each of {sorted(EXPECTED_INTERBOARD)}; seen={sorted(seen_interboard)}"))

if unexpected:
    print("PCB DRC ERROR: unexpected prototype-blocking errors:", file=sys.stderr)
    for category, block in unexpected:
        first = next((ln.strip() for ln in block.splitlines()[1:] if ln.strip()), block.strip())
        print(f"  [{category}] {first}", file=sys.stderr)
    raise SystemExit(7)

warning_counts = Counter(warnings)
fatal_warning_counts = {name: warning_counts[name] for name in FATAL_WARNINGS if warning_counts[name]}
unexpected_warning_counts = {
    name: count
    for name, count in warning_counts.items()
    if name not in FATAL_WARNINGS and (name not in WARNING_BASELINE or count > WARNING_BASELINE[name])
}
if fatal_warning_counts or unexpected_warning_counts:
    print("PCB DRC ERROR: warnings exceed the reviewed prototype baseline:", file=sys.stderr)
    if fatal_warning_counts:
        print(f"  always fatal: {fatal_warning_counts}", file=sys.stderr)
    if unexpected_warning_counts:
        print(f"  new/increased: {unexpected_warning_counts}", file=sys.stderr)
    print(f"  reviewed maximum counts: {WARNING_BASELINE}", file=sys.stderr)
    raise SystemExit(8)

print(
    "PCB DRC classification passed: only the 6 intentional inter-board disconnects remain as errors; "
    f"warnings={dict(warning_counts)} (within reviewed maxima)."
)
