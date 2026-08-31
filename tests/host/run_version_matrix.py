#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the native host regression suite against each supported ESPHome release."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VERSIONS = ("2026.8.1", "2026.8.2")


def run(cmd: list[str]) -> None:
    print("+", " ".join(str(x) for x in cmd), flush=True)
    subprocess.run(cmd, cwd=ROOT, check=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("config", "compile", "all"), default="all")
    parser.add_argument("--version", action="append", dest="versions",
                        help="ESPHome version to test; repeatable (default: 2026.8.1 + 2026.8.2)")
    parser.add_argument("--find-links", type=Path,
                        help="optional wheelhouse used with pip --no-index --find-links")
    parser.add_argument("--keep-venvs", action="store_true")
    args = parser.parse_args()

    versions = tuple(args.versions or DEFAULT_VERSIONS)
    base = ROOT / ".esphome" / "host-version-matrix" if args.keep_venvs else Path(
        tempfile.mkdtemp(prefix="unni-host-matrix-")
    )
    base.mkdir(parents=True, exist_ok=True)
    try:
        for version in versions:
            venv = base / version
            if not (venv / "bin" / "python").exists():
                run([sys.executable, "-m", "venv", str(venv)])
            python = venv / "bin" / "python"
            pip_cmd = [str(python), "-m", "pip", "install"]
            if args.find_links is not None:
                pip_cmd += ["--no-index", "--find-links", str(args.find_links)]
            run(pip_cmd + [f"esphome=={version}"])
            cmd = [str(python), "tests/host/run_host_tests.py"]
            if args.mode != "all":
                cmd += ["--mode", args.mode]
            print(f"\n=== ESPHome {version} ===", flush=True)
            run(cmd)
        print("\nPASS: ESPHome host version matrix:", ", ".join(versions))
        return 0
    finally:
        if not args.keep_venvs:
            shutil.rmtree(base, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
