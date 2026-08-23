#!/usr/bin/env python3
"""Export the two physical PCBs from the combined KiCad board as JLC-ready ZIPs.

The project intentionally keeps both PCB islands in one .kicad_pcb so the
inter-board ratsnest remains visible while designing.  Fabrication needs one
closed outline per order, so this script:

  1. creates a temporary KiCad 10.0.x compatibility copy of the 10.99 master,
  2. derives the two rectangular Edge.Cuts islands,
  3. creates one temporary PCB containing only each island and its objects,
  4. exports 4-layer Gerbers + separate PTH/NPTH Excellon drill files,
  5. creates one upload ZIP per PCB.

Usage:
  python3 tools/export_jlc_gerbers.py
  KICAD_10_0_5_CLI=/path/to/kicad-cli python3 tools/export_jlc_gerbers.py
  python3 tools/export_jlc_gerbers.py --output /tmp/jlc --keep-temp

The output ZIP contents are at ZIP root so they can be uploaded directly to
JLCPCB.  Paste layers are intentionally omitted; they belong to stencil/PCBA
outputs, not bare-PCB fabrication.
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PCB_NAME = "unni-smartification-c6.kicad_pcb"
LAYERS = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Mask,B.Mask,F.Silkscreen,B.Silkscreen,Edge.Cuts"
NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def fail(msg: str) -> None:
    raise SystemExit(f"JLC EXPORT ERROR: {msg}")


def balanced_end(text: str, start: int) -> int:
    depth = 0
    in_string = False
    escaped = False
    for i in range(start, len(text)):
        c = text[i]
        if in_string:
            if escaped:
                escaped = False
            elif c == "\\":
                escaped = True
            elif c == '"':
                in_string = False
            continue
        if c == '"':
            in_string = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i + 1
    fail(f"unterminated S-expression at offset {start}")


def top_level_forms(text: str) -> list[tuple[int, int, str]]:
    forms: list[tuple[int, int, str]] = []
    level = 0
    in_string = False
    escaped = False
    start = None
    form = None
    for i, c in enumerate(text):
        if in_string:
            if escaped:
                escaped = False
            elif c == "\\":
                escaped = True
            elif c == '"':
                in_string = False
            continue
        if c == '"':
            in_string = True
            continue
        if c == "(":
            level += 1
            if level == 2:
                start = i
                j = i + 1
                while j < len(text) and text[j].isspace():
                    j += 1
                k = j
                while k < len(text) and not text[k].isspace() and text[k] not in "()":
                    k += 1
                form = text[j:k]
        elif c == ")":
            if level == 2 and start is not None and form is not None:
                forms.append((start, i + 1, form))
                start = None
                form = None
            level -= 1
    return forms


def edge_rectangles(text: str) -> list[tuple[float, float, float, float]]:
    lines = []
    for a, b, form in top_level_forms(text):
        if form != "gr_line":
            continue
        block = text[a:b]
        if '(layer "Edge.Cuts")' not in block:
            continue
        m = re.search(rf"\(start\s+({NUM})\s+({NUM})\).*?\(end\s+({NUM})\s+({NUM})\)", block, re.S)
        if not m:
            continue
        lines.append(tuple(float(x) for x in m.groups()))
    if len(lines) not in (4, 8):
        fail(f"expected 4 or 8 rectangular Edge.Cuts lines, found {len(lines)}")

    remaining = list(lines)
    comps: list[list[tuple[float, float, float, float]]] = []
    while remaining:
        comp = [remaining.pop()]
        changed = True
        while changed:
            changed = False
            pts = {(l[0], l[1]) for l in comp} | {(l[2], l[3]) for l in comp}
            rest = []
            for ln in remaining:
                if (ln[0], ln[1]) in pts or (ln[2], ln[3]) in pts:
                    comp.append(ln)
                    changed = True
                else:
                    rest.append(ln)
            remaining = rest
        comps.append(comp)

    rects = []
    for comp in comps:
        if len(comp) != 4:
            fail(f"Edge.Cuts island has {len(comp)} edges instead of 4")
        xs = [v for l in comp for v in (l[0], l[2])]
        ys = [v for l in comp for v in (l[1], l[3])]
        rects.append((min(xs), min(ys), max(xs), max(ys)))
    return rects


def identify_boards(rects):
    usb = next((r for r in rects if math.isclose(r[2]-r[0], 19.0, abs_tol=1e-6)), None)
    esp = next((r for r in rects if math.isclose(r[2]-r[0], 46.0, abs_tol=1e-6) and math.isclose(r[3]-r[1], 32.0, abs_tol=1e-6)), None)
    if usb is None or esp is None:
        fail(f"cannot identify USB 19-mm and ESP 46x32-mm islands from {rects}")
    return {"usb-power": usb, "esp-interface": esp}


def coords_for_block(block: str, form: str) -> list[tuple[float, float]]:
    if form == "footprint":
        # 10.0.x compatibility copy uses top-level (at x y angle).
        m = re.search(rf"\n\s*\(at\s+({NUM})\s+({NUM})(?:\s+{NUM})?\)", block)
        if not m:
            # tolerate 10.99 master if someone uses --source-pcb later.
            m = re.search(rf"\(transform\s+\(translate\s+({NUM})\s+({NUM})\)", block, re.S)
        return [(float(m.group(1)), float(m.group(2)))] if m else []
    names = "start|end|mid|center|at|xy"
    return [(float(x), float(y)) for x, y in re.findall(rf"\((?:{names})\s+({NUM})\s+({NUM})(?=[\s\)])", block)]


def distance_to_rect(x: float, y: float, rect) -> float:
    dx = max(rect[0] - x, 0.0, x - rect[2])
    dy = max(rect[1] - y, 0.0, y - rect[3])
    return math.hypot(dx, dy)


def owner_for(coords, boards: dict[str, tuple[float, float, float, float]]) -> str | None:
    if not coords:
        return None
    # Average is robust for polygons/tracks; a footprint anchor is a single point.
    x = sum(p[0] for p in coords) / len(coords)
    y = sum(p[1] for p in coords) / len(coords)
    ranked = sorted((distance_to_rect(x, y, rect), name) for name, rect in boards.items())
    # Manufacturing objects should be on or very near one physical board.  A generous
    # 8 mm limit includes the 1 mm USB connector overhang while catching stale objects.
    if ranked[0][0] > 8.0:
        return None
    return ranked[0][1]


def slice_board(text: str, board_name: str, boards: dict[str, tuple[float, float, float, float]]) -> str:
    rect = boards[board_name]
    forms = top_level_forms(text)
    # Layout forms that either belong to one physical island or are editing-only.
    selectable = {
        "footprint", "segment", "arc", "via", "zone",
        "gr_line", "gr_arc", "gr_rect", "gr_circle", "gr_poly", "gr_text", "gr_text_box",
        "dimension", "target", "image",
    }
    drop_always = {"constraint", "generated", "group"}

    out = []
    last = 0
    kept_counts = {}
    for a, b, form in forms:
        out.append(text[last:a])
        block = text[a:b]
        keep = True
        if form in drop_always:
            keep = False
        elif form in selectable:
            keep = owner_for(coords_for_block(block, form), boards) == board_name
        if keep:
            out.append(block)
            kept_counts[form] = kept_counts.get(form, 0) + 1
        last = b
    out.append(text[last:])
    result = "".join(out)

    # Sanity: exactly one 4-edge rectangular outline should remain.
    rects = edge_rectangles(result)
    if len(rects) != 1:
        fail(f"{board_name}: sliced board has {len(rects)} Edge.Cuts islands")
    return result


def find_cli(explicit: str | None) -> Path:
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    for envname in ("KICAD_10_0_X_CLI", "KICAD_10_0_5_CLI"):
        if os.environ.get(envname):
            candidates.append(Path(os.environ[envname]))
    candidates += [
        Path("/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"),
        Path("/mnt/data/kicad105/AppDir/bin/kicad-cli"),
    ]
    wh = shutil.which("kicad-cli")
    if wh:
        candidates.append(Path(wh))
    for p in candidates:
        if not p.exists():
            continue
        cp = subprocess.run([str(p), "--version"], capture_output=True, text=True)
        if cp.returncode == 0 and cp.stdout.strip().startswith("10.0"):
            return p.resolve()
    fail("no KiCad 10.0.x kicad-cli found; set KICAD_10_0_X_CLI or --kicad-cli")


def run(cmd, cwd=None):
    cp = subprocess.run([str(x) for x in cmd], cwd=cwd, text=True, capture_output=True)
    if cp.returncode:
        print(cp.stdout, file=sys.stderr)
        print(cp.stderr, file=sys.stderr)
        fail("command failed: " + " ".join(str(x) for x in cmd))
    return cp


def zip_dir(src: Path, dest: Path) -> None:
    with zipfile.ZipFile(dest, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for p in sorted(src.iterdir()):
            if p.is_file() and p.suffix.lower() in {".gtl", ".g1", ".g2", ".gbl", ".gts", ".gbs", ".gto", ".gbo", ".gm1", ".drl", ".gbrjob", ".gbr"}:
                zf.write(p, p.name)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, default=ROOT / "dist" / "jlc")
    ap.add_argument("--kicad-cli")
    ap.add_argument("--keep-temp", action="store_true")
    args = ap.parse_args()
    cli = find_cli(args.kicad_cli)

    output = args.output.resolve()
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)

    tmp_obj = tempfile.TemporaryDirectory(prefix="unni-jlc-")
    tmp = Path(tmp_obj.name)
    compat = tmp / "compat"
    converter = ROOT / "tools" / "make_kicad_10_0_5_copy.py"
    run([sys.executable, converter, "--source", ROOT, "--output", compat])
    pcb = compat / PCB_NAME
    text = pcb.read_text(encoding="utf-8")
    boards = identify_boards(edge_rectangles(text))

    summary = []
    for name, rect in boards.items():
        stem = f"unni-{name}"
        work = tmp / name
        work.mkdir()
        sliced = work / f"{stem}.kicad_pcb"
        sliced.write_text(slice_board(text, name, boards), encoding="utf-8")

        # Let stable KiCad refill/save the isolated board before plotting.
        run([cli, "pcb", "drc", "--refill-zones", "--save-board", "-o", work / "drc.rpt", sliced])

        fab = output / name
        fab.mkdir()
        run([cli, "pcb", "export", "gerbers", "--layers", LAYERS, "--check-zones", "-o", fab, sliced])
        run([
            cli, "pcb", "export", "drill", "--format", "excellon",
            "--excellon-separate-th", "--excellon-units", "mm", "-o", fab, sliced,
        ])

        # Never upload the temporary board or DRC report as fabrication data.
        archive = output / f"{stem}-jlc-gerbers.zip"
        zip_dir(fab, archive)
        if archive.stat().st_size == 0:
            fail(f"empty archive for {name}")
        summary.append((name, rect, archive))

    (output / "README.txt").write_text(
        "JLCPCB fabrication exports\n"
        "==========================\n\n"
        "Generated from the combined two-island KiCad 10.99 master via a temporary KiCad 10.0.x compatibility copy.\n"
        "Each ZIP contains one physical PCB only: four copper layers, solder masks, silkscreens, Edge.Cuts and separate PTH/NPTH Excellon drill files.\n"
        "F.Paste/B.Paste are intentionally omitted because they are stencil/assembly data, not bare-PCB fabrication layers.\n"
        "Regenerate these files after every PCB change; do not edit the sliced temporary boards.\n"
    )

    print(f"KiCad CLI: {cli}")
    for name, rect, archive in summary:
        print(f"{name}: outline {rect[2]-rect[0]:g} x {rect[3]-rect[1]:g} mm -> {archive}")
    if args.keep_temp:
        keep = output / "_debug-temp"
        shutil.copytree(tmp, keep)
        print(f"temporary split boards copied to {keep}")
    tmp_obj.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
