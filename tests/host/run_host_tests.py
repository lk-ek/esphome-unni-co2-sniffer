#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build and run every shipped UNNI YAML feature variant on ESPHome host.

The generated host YAML is derived mechanically from the real device YAML:
ESP32/network/radio-only top-level blocks are removed, while the complete
co2_monitor_0601 block is retained. This keeps the host matrix tied to the
shipped configurations instead of maintaining five independent copies.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import selectors
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
GENERATED_PREFIX = ".host-test-"
PASS_MARKER = "UNNI HOST SELF-TEST PASSED"

VARIANTS = (
    "i2c-sniffer.yaml",
    "i2c-sniffer-debug.yaml",
    "i2c-sniffer-no-ble.yaml",
    "i2c-sniffer-ble-only.yaml",
    "i2c-sniffer-sht43-probe.yaml",
)

# These components either do not exist on host or would test ESP32/network
# behavior rather than the portable UNNI component. logger, esphome and the
# external component declaration are intentionally retained.
REMOVE_TOP_LEVEL = {
    "api",
    "ota",
    "wifi",
    "captive_portal",
    "esp32_ble",
    "esp32",
}

TOP_LEVEL_KEY = re.compile(r"^([a-zA-Z0-9_]+):(?:\s|$)")


def _drop_top_level_blocks(text: str) -> str:
    output: list[str] = []
    dropping = False
    for line in text.splitlines(keepends=True):
        match = TOP_LEVEL_KEY.match(line)
        if match:
            dropping = match.group(1) in REMOVE_TOP_LEVEL
        if not dropping:
            output.append(line)
    return "".join(output)


def _set_logger_debug(text: str) -> str:
    lines = text.splitlines()
    in_logger = False
    for index, line in enumerate(lines):
        if line.startswith("logger:"):
            in_logger = True
            continue
        if in_logger and line and not line.startswith((" ", "#")):
            break
        if in_logger and re.match(r"^  level:\s*", line):
            lines[index] = "  level: DEBUG"
            return "\n".join(lines) + "\n"
    raise RuntimeError("could not force logger.level=DEBUG for host smoke test")


def _set_host_name(text: str, variant: str) -> str:
    name = "unni-host-" + variant.removesuffix(".yaml").replace("i2c-sniffer", "base").replace("_", "-")
    lines = text.splitlines()
    in_esphome = False
    replaced = False
    for index, line in enumerate(lines):
        if line.startswith("esphome:"):
            in_esphome = True
            continue
        if in_esphome and line and not line.startswith((" ", "#")):
            in_esphome = False
        if in_esphome and re.match(r"^  name:\s*", line):
            lines[index] = f"  name: {name}"
            replaced = True
            break
    if not replaced:
        raise RuntimeError(f"{variant}: could not replace esphome.name")
    return "\n".join(lines) + "\n"


def _ensure_cpp20_build_flag(text: str) -> str:
    """Explicitly request C++20 for generated host configs.

    ESPHome's host platform also adds -std=gnu++20 itself. Keeping the flag in
    the generated test YAML makes the host test's compiler requirement explicit
    and protects the test runner from native-toolchain/default-standard quirks.
    """
    lines = text.splitlines()
    insert_at = None
    in_esphome = False
    for index, line in enumerate(lines):
        if line.startswith("esphome:"):
            in_esphome = True
            continue
        if in_esphome and line and not line.startswith((" ", "#")):
            insert_at = index
            break
    if insert_at is None:
        insert_at = len(lines)
    lines[insert_at:insert_at] = [
        "  # ESPHome host requires C++20 (concepts/std::span).",
        "  build_flags:",
        '    - "-std=gnu++20"',
    ]
    return "\n".join(lines) + "\n"


def _compiler_preflight() -> tuple[bool, str]:
    """Check that the native C++ compiler can actually compile C++20 concepts."""
    compiler = os.environ.get("CXX") or shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        return False, "no native C++ compiler found (CXX/c++/clang++/g++)"

    probe = ROOT / ".esphome" / "host-cpp20-probe.cpp"
    binary = ROOT / ".esphome" / "host-cpp20-probe"
    probe.parent.mkdir(parents=True, exist_ok=True)
    probe.write_text(
        "#include <concepts>\n"
        "#include <span>\n"
        "template<std::integral T> constexpr T twice(T v) { return v * 2; }\n"
        "int main() { int a[2] = {1, 2}; std::span<int> s(a); return twice(s[0]) == 2 ? 0 : 1; }\n",
        encoding="utf-8",
    )
    try:
        proc = subprocess.run(
            [compiler, "-std=gnu++20", str(probe), "-o", str(binary)],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if proc.returncode != 0:
            return False, f"{compiler} cannot compile the required C++20 features:\n{proc.stdout.strip()}"
        version = subprocess.run(
            [compiler, "--version"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        ).stdout.splitlines()
        return True, f"{compiler}: {version[0] if version else 'C++20 probe passed'}"
    finally:
        probe.unlink(missing_ok=True)
        binary.unlink(missing_ok=True)


def make_host_config(source: Path) -> Path:
    text = _drop_top_level_blocks(source.read_text(encoding="utf-8"))
    text = _set_host_name(text, source.name)
    text = _set_logger_debug(text)
    text = _ensure_cpp20_build_flag(text)
    # Host networking is supplied by the OS; no wifi: block is valid/needed.
    text += "\nhost:\n  mac_address: \"02:00:00:00:06:01\"\n"
    generated = ROOT / f"{GENERATED_PREFIX}{source.name}"
    generated.write_text(text, encoding="utf-8")
    return generated


def _run_command(command: list[str], env: dict[str, str]) -> int:
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, cwd=ROOT, env=env, check=False).returncode


def _run_smoke(esphome: str, config: Path, env: dict[str, str], timeout_s: int) -> bool:
    command = [esphome, "run", config.name]
    print("+", " ".join(command), flush=True)
    proc = subprocess.Popen(
        command,
        cwd=ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    deadline = time.monotonic() + timeout_s
    passed = False
    try:
        assert proc.stdout is not None
        selector = selectors.DefaultSelector()
        selector.register(proc.stdout, selectors.EVENT_READ)
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            events = selector.select(timeout=min(0.25, remaining))
            if events:
                line = proc.stdout.readline()
                if line:
                    print(line, end="")
                    if PASS_MARKER in line:
                        passed = True
                        break
            if proc.poll() is not None:
                # Drain any final buffered lines before deciding the marker was absent.
                for line in proc.stdout:
                    print(line, end="")
                    if PASS_MARKER in line:
                        passed = True
                break
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
    return passed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("config", "compile", "run"),
        default="run",
        help="validate, native-compile, or compile+execute each host variant (default: run)",
    )
    parser.add_argument("--timeout", type=int, default=300, help="seconds per host run")
    parser.add_argument("--keep-generated", action="store_true")
    parser.add_argument("--esphome", default=shutil.which("esphome") or "esphome")
    args = parser.parse_args()

    ok, compiler_info = _compiler_preflight()
    print(f"Native compiler preflight: {compiler_info}", flush=True)
    if not ok:
        print(
            "\nHost tests require a native C++20 compiler. ESPHome 2026.7.x host "
            "uses C++20 features such as std::integral, std::convertible_to and std::span. "
            "Update/select the macOS Command Line Tools or set CXX to a C++20-capable compiler.",
            file=sys.stderr,
        )
        return 2

    failures: list[str] = []
    generated_files: list[Path] = []
    try:
        for variant in VARIANTS:
            source = ROOT / variant
            if not source.exists():
                failures.append(f"{variant}: source YAML missing")
                continue

            generated = make_host_config(source)
            generated_files.append(generated)
            print(f"\n=== {variant} -> host ===", flush=True)

            env = os.environ.copy()
            # ESPHome host supports an isolated preferences directory. Keep test
            # state out of the user's normal host preferences and out of HOME.
            prefd = ROOT / ".esphome" / "host-test-prefs" / source.stem
            prefd.mkdir(parents=True, exist_ok=True)
            env["ESPHOME_PREFDIR"] = str(prefd)

            if args.mode == "config":
                ok = _run_command([args.esphome, "config", generated.name], env) == 0
            elif args.mode == "compile":
                ok = _run_command([args.esphome, "compile", generated.name], env) == 0
            else:
                ok = _run_smoke(args.esphome, generated, env, args.timeout)

            if not ok:
                failures.append(f"{variant}: {args.mode} failed")

        if failures:
            print("\nHost test failures:", file=sys.stderr)
            for failure in failures:
                print(f"  - {failure}", file=sys.stderr)
            return 1

        print(f"\nPASS: all {len(VARIANTS)} YAML variants passed host {args.mode} tests")
        return 0
    finally:
        if not args.keep_generated:
            for generated in generated_files:
                generated.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
