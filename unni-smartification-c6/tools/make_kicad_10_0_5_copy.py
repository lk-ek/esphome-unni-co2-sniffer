#!/usr/bin/env python3
"""Create a KiCad 10.0.5-compatible project copy from the KiCad 10.99 master.

The 10.99 master intentionally uses features that KiCad 10.0.5 cannot parse:
  * PCB geometric constraint objects
  * PCB generated via-stitch objects
  * footprint-level transform blocks
  * 10.99/11-development file-format headers

The compatibility export preserves the actual instantiated copper/vias/footprints and
removes only the 10.99 editing metadata.  The generated copy is intended for stable
KiCad 10.0.5 tooling/plugins and fabrication export; edit the 10.99 master instead.

Usage:
  python3 tools/make_kicad_10_0_5_copy.py
  python3 tools/make_kicad_10_0_5_copy.py --output /tmp/unni-kicad-10.0.5
  KICAD_10_0_5_CLI=/path/to/kicad-cli python3 tools/make_kicad_10_0_5_copy.py --validate
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

PCB_TARGET_VERSION = "20260206"
SCH_TARGET_VERSION = "20260306"
TARGET_GENERATOR_VERSION = "10.0"
PROJECT_BASENAME = "unni-smartification-c6"


def _balanced_end(text: str, start: int) -> int:
    """Return one-past-end of the S-expression beginning at *start*."""
    if text[start] != "(":
        raise ValueError(f"expected '(' at offset {start}")
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
    raise ValueError(f"unterminated S-expression at offset {start}")


def _remove_top_level_forms(text: str, form: str) -> tuple[str, list[str], int]:
    """Remove tab-indented top-level PCB forms and return their UUIDs."""
    pattern = re.compile(rf"\n\t\({re.escape(form)}(?:\s|\n)")
    starts = [m.start() + 2 for m in pattern.finditer(text)]  # points at '('
    removed_uuids: list[str] = []
    for start in reversed(starts):
        end = _balanced_end(text, start)
        block = text[start:end]
        m = re.search(r'\(uuid\s+"([0-9a-fA-F-]+)"\)', block)
        if m:
            removed_uuids.append(m.group(1))
        # consume preceding tab/newline cleanly; retain exactly one newline
        line_start = text.rfind("\n", 0, start)
        text = text[:line_start] + text[end:]
    return text, removed_uuids, len(starts)


def _drop_dangling_group_members(text: str, uuids: list[str]) -> str:
    # Removed 10.99 constraint/generated objects may have been included in KiCad groups.
    # Delete only quoted UUID references; object UUID declarations are already gone.
    for uid in uuids:
        text = text.replace(f' "{uid}"', "")
        text = text.replace(f'\n\t\t\t"{uid}"', "")
    return text




def _strip_design_block_links_from_groups(text: str) -> tuple[str, int]:
    """Turn linked design-block groups into ordinary groups for KiCad 10.0.x.

    KiCad 10.99 stores a design-block origin on (group ...) objects via
    (lib_id "Library:..."). Stable 10.0.x follows that link and may try to
    open the external .kicad_block/.kicad_pcb with a newer file format.
    The instantiated group members are already fully present in the project,
    so a compatibility export should preserve the group and remove only the
    design-block library link.
    """
    pattern = re.compile(r"\n\t\(group(?:\s|\n)")
    starts = [m.start() + 2 for m in pattern.finditer(text)]
    count = 0
    for start in reversed(starts):
        end = _balanced_end(text, start)
        block = text[start:end]
        block2, n = re.subn(r'\n(?P<i>\s*)\(lib_id\s+"[^"]+"\)', '', block)
        if n:
            text = text[:start] + block2 + text[end:]
            count += n
    return text, count

def _convert_footprint_transforms(text: str) -> tuple[str, int]:
    """Convert 10.99 footprint (transform ...) placement to 10.0.x (at ...)."""
    # Current 10.99 project uses unit scale only. Refuse non-unit scale instead of
    # silently corrupting geometry.
    transform_re = re.compile(
        r"\n(?P<indent>\s*)\(transform\s*\n"
        r"(?P=indent)\t\(translate\s+(?P<x>[-+0-9.eE]+)\s+(?P<y>[-+0-9.eE]+)\)\s*\n"
        r"(?P=indent)\t\(rotate\s+(?P<a>[-+0-9.eE]+)\)\s*\n"
        r"(?P=indent)\t\(scale\s+(?P<sx>[-+0-9.eE]+)\s+(?P<sy>[-+0-9.eE]+)\)\s*\n"
        r"(?P=indent)\)"
    )

    count = 0

    def repl(m: re.Match[str]) -> str:
        nonlocal count
        sx, sy = float(m.group("sx")), float(m.group("sy"))
        if abs(sx - 1.0) > 1e-12 or abs(sy - 1.0) > 1e-12:
            raise ValueError(
                f"non-unit footprint scale {sx:g},{sy:g} is not safely convertible"
            )
        count += 1
        return (
            f'\n{m.group("indent")}(at {m.group("x")} {m.group("y")} {m.group("a")})'
        )

    out = transform_re.sub(repl, text)
    if "\n\t\t(transform\n" in out:
        raise ValueError("unconverted footprint transform remains")
    return out, count


def convert_pcb(path: Path) -> dict[str, int]:
    text = path.read_text(encoding="utf-8")
    original_vias = len(re.findall(r"\n\t\(via(?:\s|\n)", text))

    text, n_constraints_ids, n_constraints = _remove_top_level_forms(text, "constraint")
    text, n_generated_ids, n_generated = _remove_top_level_forms(text, "generated")
    text = _drop_dangling_group_members(text, n_constraints_ids + n_generated_ids)
    text, n_design_block_links = _strip_design_block_links_from_groups(text)
    text, n_transforms = _convert_footprint_transforms(text)

    # KiCad 10.99 adds frequency-dependent dielectric metadata that stable 10.0
    # does not understand. Preserve thickness/material/epsilon_r/loss_tangent and
    # drop only these two nightly-only fields.
    text, n_specfreq = re.subn(r"\n\s*\(spec_frequency\s+[^)]+\)", "", text)
    text, n_dmodel = re.subn(r"\n\s*\(dielectric_model\s+[^)]+\)", "", text)

    text, nver = re.subn(r"\(version\s+\d+\)", f"(version {PCB_TARGET_VERSION})", text, count=1)
    text, ngen = re.subn(
        r'\(generator_version\s+"[^"]+"\)',
        f'(generator_version "{TARGET_GENERATOR_VERSION}")',
        text,
        count=1,
    )
    if nver != 1 or ngen != 1:
        raise ValueError(f"failed to rewrite PCB header in {path}")

    converted_vias = len(re.findall(r"\n\t\(via(?:\s|\n)", text))
    if converted_vias != original_vias:
        raise ValueError(
            f"explicit via count changed {original_vias} -> {converted_vias}; refusing export"
        )

    path.write_text(text, encoding="utf-8")
    return {
        "constraints_removed": n_constraints,
        "generated_via_stitch_removed": n_generated,
        "design_block_group_links_removed": n_design_block_links,
        "footprint_transforms_converted": n_transforms,
        "explicit_vias_preserved": converted_vias,
        "spec_frequency_removed": n_specfreq,
        "dielectric_model_removed": n_dmodel,
    }


def convert_schematic(path: Path) -> dict[str, int]:
    text = path.read_text(encoding="utf-8")

    # Stable KiCad 10 understands body_style, passthrough, in_pos_files, etc.
    # Do NOT strip those (that would be a KiCad 10 -> 9 conversion). Only drop
    # constructs introduced on the 10.99/11 development line.
    removed = {}
    text, n_design_block_links = _strip_design_block_links_from_groups(text)
    removed["design_block_group_links"] = n_design_block_links
    for form in ("ellipse", "ellipse_arc", "net_chain", "net_chains"):
        text, _ids, n = _remove_top_level_forms(text, form)
        removed[form] = n
    text, n_locked = re.subn(r"\n\s*\(locked\s+(?:yes|no)\)", "", text)
    removed["locked"] = n_locked

    text, nver = re.subn(r"\(version\s+\d+\)", f"(version {SCH_TARGET_VERSION})", text, count=1)
    text, ngen = re.subn(
        r'\(generator_version\s+"[^"]+"\)',
        f'(generator_version "{TARGET_GENERATOR_VERSION}")',
        text,
        count=1,
    )
    if nver != 1 or ngen != 1:
        raise ValueError(f"failed to rewrite schematic header in {path}")
    path.write_text(text, encoding="utf-8")
    return removed


def assert_no_linked_design_block_groups(root: Path) -> None:
    """Fail if a converted KiCad file still contains a linked group lib_id."""
    leftovers: list[str] = []
    for path in [*root.rglob("*.kicad_pcb"), *root.rglob("*.kicad_sch")]:
        text = path.read_text(encoding="utf-8")
        pattern = re.compile(r"\n\t\(group(?:\s|\n)")
        for m in pattern.finditer(text):
            start = m.start() + 2
            end = _balanced_end(text, start)
            block = text[start:end]
            if re.search(r'\(lib_id\s+"[^"]+"\)', block):
                leftovers.append(str(path.relative_to(root)))
                break
    if leftovers:
        raise ValueError("linked design-block group metadata remains in: " + ", ".join(leftovers))


def sanitize_project_local_state(root: Path) -> None:
    # Local GUI state from 10.99 can contain settings unknown to the stable build and
    # has no place in a reproducible compatibility export.
    for pattern in ("*.kicad_prl", "*.lck", "~*.kicad_*", "*.before-netfix"):
        for p in root.rglob(pattern):
            if p.is_file():
                p.unlink()


def find_cli(explicit: str | None) -> Path | None:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    env = __import__("os").environ.get("KICAD_10_0_5_CLI")
    if env:
        candidates.append(Path(env))
    # Common stable installation locations. Do not accept a different major/minor
    # silently; version is checked below.
    candidates += [
        Path("/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"),
        Path("/usr/bin/kicad-cli"),
        Path("/usr/local/bin/kicad-cli"),
    ]
    for p in candidates:
        if p.exists() and p.is_file():
            return p
    found = shutil.which("kicad-cli")
    return Path(found) if found else None


def run(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)


def validate_with_10_0_5(root: Path, cli: Path) -> None:
    ver = run([str(cli), "--version"], root)
    if ver.returncode != 0:
        raise RuntimeError(ver.stderr or ver.stdout)
    version = ver.stdout.strip()
    if not version.startswith("10.0"):
        
        raise RuntimeError(f"expected KiCad 10.0.x CLI, got {version!r} from {cli}")

    val = root / "validation-kicad-10.0.5"
    if val.exists():
        shutil.rmtree(val)
    val.mkdir()

    pcb = root / f"{PROJECT_BASENAME}.kicad_pcb"
    sch = root / f"{PROJECT_BASENAME}.kicad_sch"

    checks = [
        ([str(cli), "sch", "erc", "-o", str(val / "erc.rpt"), str(sch)], "ERC"),
        ([str(cli), "sch", "export", "netlist", "--format", "kicadxml", "-o", str(val / "netlist.xml"), str(sch)], "netlist"),
        ([str(cli), "pcb", "drc", "--schematic-parity", "-o", str(val / "drc.rpt"), str(pcb)], "DRC/parity"),
        ([str(cli), "pcb", "export", "gerbers", "-o", str(val / "gerbers"), str(pcb)], "Gerber export"),
        ([str(cli), "pcb", "export", "drill", "-o", str(val / "drill"), str(pcb)], "drill export"),
    ]
    for cmd, label in checks:
        cp = run(cmd, root)
        # ERC/DRC return zero as long as --exit-code-violations is not requested.
        if cp.returncode != 0:
            raise RuntimeError(
                f"KiCad 10.0.5 {label} failed (rc={cp.returncode})\nSTDOUT:\n{cp.stdout}\nSTDERR:\n{cp.stderr}"
            )

    (val / "kicad-version.txt").write_text(version + "\n")
    (val / "README.txt").write_text(
        "Generated by make_kicad_10_0_5_copy.py.\n"
        "Successful parse/ERC/netlist/DRC+schematic-parity/Gerber/drill smoke test with KiCad 10.0.5.\n"
        "DRC/ERC reports may contain expected unfinished-layout violations; conversion validity is the smoke-test criterion.\n"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument(
        "--output",
        type=Path,
        default=None,
        help="output directory; default: <source>/dist/kicad-10.0.5/unni-smartification-c6",
    )
    ap.add_argument("--validate", action="store_true", help="smoke-test result with an actual KiCad 10.0.5 CLI")
    ap.add_argument("--kicad-cli", help="explicit KiCad 10.0.5 kicad-cli path")
    args = ap.parse_args()

    source = args.source.resolve()
    output = (args.output or (source / "dist" / "kicad-10.0.5" / PROJECT_BASENAME)).resolve()
    if source == output or source in output.parents and output == source:
        raise SystemExit("output must differ from source")

    # Prevent recursively copying a previous dist tree into the next export.
    def ignore(src: str, names: list[str]) -> set[str]:
        ignored = {"dist"} if Path(src).resolve() == source else set()
        ignored |= {n for n in names if n in {".DS_Store", "__MACOSX"}}
        return ignored

    if output.exists():
        shutil.rmtree(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, output, ignore=ignore)
    sanitize_project_local_state(output)

    stats: dict[str, dict[str, int]] = {}
    pcbs = list(output.rglob("*.kicad_pcb"))
    if not pcbs:
        raise SystemExit("no .kicad_pcb found")
    for p in pcbs:
        stats[str(p.relative_to(output))] = convert_pcb(p)
    for p in output.rglob("*.kicad_sch"):
        convert_schematic(p)
    assert_no_linked_design_block_groups(output)

    # Marker and machine-readable conversion manifest.
    manifest = {
        "target": "KiCad 10.0.5",
        "source": "KiCad 10.99 master",
        "pcb_target_format": PCB_TARGET_VERSION,
        "schematic_target_format": SCH_TARGET_VERSION,
        "pcb_conversion": stats,
        "notes": [
            "10.99 geometric constraints removed; geometry is preserved and checked by project validators.",
            "10.99 generated via-stitch descriptors removed; already-instantiated explicit vias are preserved.",
            "10.99 footprint transforms converted to KiCad 10.0.x footprint placements.",
            "10.99 stackup spec_frequency/dielectric_model metadata removed; physical stackup values preserved.",
            "Design-block lib_id links on instantiated groups removed; group contents remain local and intact.",
            "Do not edit this compatibility tree as the project master.",
        ],
    }
    (output / "KICAD_10_0_5_COMPATIBILITY.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (output / "KICAD_10_0_5_COMPATIBILITY.txt").write_text(
        "KiCad 10.0.5 compatibility export\n"
        "=================================\n\n"
        "Generated from the KiCad 10.99 master by tools/make_kicad_10_0_5_copy.py.\n"
        "Use this tree for KiCad 10.0.5-only plugins and fabrication export.\n"
        "Make design edits in the 10.99 master and regenerate this copy.\n\n"
        "Removed editing-only 10.99 features:\n"
        "- native PCB geometric constraint objects\n"
        "- generated via-stitch descriptors (explicit generated vias remain)\n"
        "- 10.99 footprint transform wrappers (converted to KiCad 10 placements)\n"
        "- 10.99 stackup frequency/model metadata (thickness/material/Dk preserved)\n"
        "- design-block library links on instantiated groups (group contents preserved)\n"
    )

    cli = find_cli(args.kicad_cli)
    if args.validate:
        if cli is None:
            raise SystemExit("--validate requested but no kicad-cli found; set KICAD_10_0_5_CLI or --kicad-cli")
        validate_with_10_0_5(output, cli)
        print(f"KiCad 10.0.5 smoke validation passed with {cli}")
    elif cli is not None:
        # Opportunistically tell the user whether the discovered CLI is the right one,
        # but do not make conversion depend on a local install.
        cp = run([str(cli), "--version"], output)
        if cp.returncode == 0 and cp.stdout.strip().startswith("10.0"):
            print(f"KiCad 10.0.x CLI found at {cli}; use --validate to smoke-test the export.")

    print(f"Created KiCad 10.0.5 compatibility project: {output}")
    for name, st in stats.items():
        print(
            f"  {name}: {st['constraints_removed']} constraints removed, "
            f"{st['generated_via_stitch_removed']} generated via-stitch descriptors removed, "
            f"{st['design_block_group_links_removed']} design-block group links removed, "
            f"{st['footprint_transforms_converted']} footprint transforms converted, "
            f"{st['explicit_vias_preserved']} explicit vias preserved"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
