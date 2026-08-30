#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build and run every shipped YAML feature variant on ESPHome host.

The generated host YAML is derived mechanically from the real device YAML:
ESP32/network/radio-only top-level blocks are removed, while the complete
component blocks are retained. This keeps the host matrix tied to the shipped
configurations instead of maintaining independent copies.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
FAIL_MARKER = "UNNI HOST SELF-TEST FAILED"
BRIDGE_PASS_MARKER = "SENSIRION BRIDGE HOST SELF-TEST PASSED"
BRIDGE_FAIL_MARKER = "SENSIRION BRIDGE HOST SELF-TEST FAILED"

VARIANTS = (
    "i2c-sniffer.yaml",
    "i2c-sniffer-debug.yaml",
    "i2c-sniffer-no-ble.yaml",
    "i2c-sniffer-ble-only.yaml",
    "i2c-sniffer-sht43-probe.yaml",
    "mobilesensor-sensirion.yaml",
    "mobilesensor-sensirion-no-ens160.yaml",
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
    "time",
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


def _top_level_block(text: str, key: str) -> str:
    lines = text.splitlines(keepends=True)
    output: list[str] = []
    copying = False
    for line in lines:
        match = TOP_LEVEL_KEY.match(line)
        if match:
            if copying:
                break
            copying = match.group(1) == key
        if copying:
            output.append(line)
    if not output:
        raise RuntimeError(f"top-level block {key!r} not found")
    return "".join(output)


def _mobile_host_config(text: str) -> str:
    # The real mobile YAML's AHT21/ENS160, raw I2C control and boot automation
    # are hardware-only. Retain the exact bridge block while replacing those
    # sensors with portable templates carrying the same source/entity IDs.
    bridge = _top_level_block(text, "sensirion_gadget_bridge")
    ens160_templates = """
  - platform: template
    id: ens160_tvoc
    name: ENS160 TVOC
  - platform: template
    id: ens160_aqi
    name: ENS160 AQI
""" if "platform: ens160_i2c" in text else ""
    return f"""esphome:
  name: mobilesensor-host
  friendly_name: mobilesensor host

external_components:
  - source:
      type: local
      path: .

logger:
  level: DEBUG

sensor:
  - platform: template
    id: aht21_temp
    name: AHT21 Temperature
  - platform: template
    id: aht21_humi
    name: AHT21 Humidity
{ens160_templates}

{bridge}"""


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


def _host_name(variant: str) -> str:
    if variant == "mobilesensor-sensirion.yaml":
        return "unni-host-mobile-sensirion"
    if variant == "mobilesensor-sensirion-no-ens160.yaml":
        return "unni-host-mobile-aht21"
    return "unni-host-" + variant.removesuffix(".yaml").replace("i2c-sniffer", "base").replace("_", "-")


def _set_host_name(text: str, variant: str) -> str:
    name = _host_name(variant)
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


@dataclass(frozen=True)
class NativeToolchain:
    compiler: str
    compile_flags: tuple[str, ...]
    description: str


def _yaml_quote(value: str) -> str:
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"') + '"'


def _inject_host_build_flags(text: str, flags: tuple[str, ...]) -> str:
    """Add host-only native toolchain flags to the generated configuration."""
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

    block = [
        "  # Native host test toolchain; generated by tests/host/run_host_tests.py.",
        "  build_flags:",
    ]
    block.extend(f"    - {_yaml_quote(flag)}" for flag in flags)
    lines[insert_at:insert_at] = block
    return "\n".join(lines) + "\n"


def _probe_compiler(compiler: str, flags: tuple[str, ...]) -> tuple[bool, str]:
    """Compile a tiny C++20 program with exactly the flags used by host builds."""
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
            [compiler, *flags, str(probe), "-o", str(binary)],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if proc.returncode != 0:
            return False, proc.stdout.strip()
        run = subprocess.run(
            [str(binary)],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if run.returncode != 0:
            return False, f"probe binary exited with {run.returncode}: {run.stdout.strip()}"
        return True, ""
    finally:
        probe.unlink(missing_ok=True)
        binary.unlink(missing_ok=True)


def _compiler_version(compiler: str) -> str:
    version = subprocess.run(
        [compiler, "--version"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    ).stdout.splitlines()
    return version[0] if version else compiler


def _homebrew_llvm_prefix() -> Path | None:
    brew = shutil.which("brew")
    if not brew:
        return None
    proc = subprocess.run(
        [brew, "--prefix", "llvm"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        return None
    prefix = Path(proc.stdout.strip())
    if not prefix:
        return None
    return prefix


def _select_native_toolchain() -> tuple[NativeToolchain | None, str]:
    """Select a C++20 toolchain and flags usable by PlatformIO native.

    On macOS the compiler and libc++ are deliberately treated as a pair. Some
    Apple Command Line Tools releases accept -std=gnu++20 but ship a libc++ that
    lacks the C++20 concepts used by ESPHome. In that case use Homebrew LLVM's
    libc++ headers and runtime explicitly. The flags returned here are injected
    into the generated ESPHome host YAML, so the preflight and real build use
    the same C++ standard-library selection.
    """
    base_flags = ("-std=gnu++20",)
    diagnostics: list[str] = []

    # Respect an explicit compiler first, but on macOS do not assume that CXX
    # also makes PlatformIO select the matching C++ standard library.
    explicit = os.environ.get("CXX")
    default_compiler = explicit or shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")

    if sys.platform == "darwin":
        brew_prefix = _homebrew_llvm_prefix()
        if brew_prefix is not None:
            brew_compiler = brew_prefix / "bin" / "clang++"
            brew_include = brew_prefix / "include" / "c++" / "v1"
            brew_lib = brew_prefix / "lib" / "c++"
            if brew_compiler.exists() and brew_include.is_dir() and brew_lib.is_dir():
                brew_flags = (
                    "-std=gnu++20",
                    "-nostdinc++",
                    f"-I{brew_include}",
                    f"-L{brew_lib}",
                    f"-Wl,-rpath,{brew_lib}",
                )
                # PlatformIO native may ignore CXX and invoke the system compiler.
                # Therefore validate the exact compiler that PlatformIO is likely to use
                # with Homebrew's libc++ flags first. A modern Apple clang can compile
                # against Homebrew libc++; the important part here is the standard library.
                pio_compiler = shutil.which("c++") or shutil.which("clang++") or str(brew_compiler)
                ok, error = _probe_compiler(pio_compiler, brew_flags)
                if ok:
                    return (
                        NativeToolchain(
                            compiler=pio_compiler,
                            compile_flags=brew_flags,
                            description=(
                                f"{_compiler_version(pio_compiler)} with Homebrew libc++ "
                                f"from {brew_prefix}"
                            ),
                        ),
                        "",
                    )
                diagnostics.append(
                    f"System compiler + Homebrew libc++ probe failed:\n{error}"
                )

                # Also probe Homebrew clang itself. This is useful diagnostic evidence,
                # but we do not rely on CXX alone because PlatformIO/native may still use
                # the system compiler internally.
                ok, error = _probe_compiler(str(brew_compiler), brew_flags)
                if not ok:
                    diagnostics.append(f"Homebrew LLVM probe failed:\n{error}")

        if default_compiler:
            ok, error = _probe_compiler(default_compiler, base_flags)
            if ok:
                return (
                    NativeToolchain(
                        compiler=default_compiler,
                        compile_flags=base_flags,
                        description=_compiler_version(default_compiler),
                    ),
                    "",
                )
            diagnostics.append(f"Apple/default native toolchain probe failed:\n{error}")
    else:
        if default_compiler:
            ok, error = _probe_compiler(default_compiler, base_flags)
            if ok:
                return (
                    NativeToolchain(
                        compiler=default_compiler,
                        compile_flags=base_flags,
                        description=_compiler_version(default_compiler),
                    ),
                    "",
                )
            diagnostics.append(f"Native toolchain probe failed:\n{error}")

    if not default_compiler and sys.platform != "darwin":
        diagnostics.append("no native C++ compiler found (CXX/c++/clang++/g++)")
    return None, "\n\n".join(diagnostics)

def make_host_config(source: Path, toolchain: NativeToolchain) -> Path:
    source_text = source.read_text(encoding="utf-8")
    text = (_mobile_host_config(source_text) if source.name.startswith("mobilesensor-sensirion")
            else _drop_top_level_blocks(source_text))
    text = _set_host_name(text, source.name)
    text = _set_logger_debug(text)
    text = _inject_host_build_flags(text, toolchain.compile_flags)
    # Host networking is supplied by the OS; no wifi: block is valid/needed.
    text += "\nhost:\n  mac_address: \"02:00:00:00:06:01\"\n"
    generated = ROOT / f"{GENERATED_PREFIX}{source.name}"
    generated.write_text(text, encoding="utf-8")
    return generated


def make_negative_schema_configs(toolchain: NativeToolchain) -> list[tuple[Path, str]]:
    common = """esphome:
  name: {name}
  build_flags:
{flags}
external_components:
  - source:
      type: local
      path: .
logger:
  level: DEBUG
sensor:
  - platform: template
    id: source_temperature
    name: Source Temperature
  - platform: template
    id: source_humidity
    name: Source Humidity
{component}
host:
  mac_address: "02:00:00:00:06:02"
"""
    flags = "\n".join(f"    - {_yaml_quote(flag)}" for flag in toolchain.compile_flags)
    cases = {
        "missing-source-pair": (
            "neg-missing-pair",
            "temperature_id and humidity_id must be configured together",
            """sensirion_gadget_bridge:
  ble: false
  ble_live: false
  ble_history: false
  profile: sht43_trh
  temperature_id: source_temperature
""",
        ),
        "legacy-sources-without-standalone": (
            "neg-source-mode",
            "Sensirion source IDs require standalone_sensirion_mode: true",
            """sensirion_gadget_bridge:
  id: schema_bridge
  ble: false
  ble_live: false
  ble_history: false
co2_monitor_0601:
  sensirion_bridge_id: schema_bridge
  ble: false
  home_assistant: false
  sensirion_temperature_id: source_temperature
  sensirion_humidity_id: source_humidity
""",
        ),
        "conflicting-profile-alias": (
            "neg-profile-alias",
            "sht43_identity_probe: true conflicts with profile: trh_co2",
            """sensirion_gadget_bridge:
  ble: false
  ble_live: false
  ble_history: false
  profile: trh_co2
  sht43_identity_probe: true
""",
        ),
        "legacy-missing-source-pair": (
            "neg-legacy-pair",
            "sensirion_temperature_id and sensirion_humidity_id must be configured together",
            """sensirion_gadget_bridge:
  id: schema_bridge
  ble: false
  ble_live: false
  ble_history: false
co2_monitor_0601:
  sensirion_bridge_id: schema_bridge
  ble: false
  home_assistant: false
  standalone_sensirion_mode: true
  sensirion_profile: sht43_trh
  sensirion_temperature_id: source_temperature
  sniffer_enabled: false
  rtrh_enabled: false
""",
        ),
    }
    paths: list[tuple[Path, str]] = []
    for suffix, (name, expected_error, component) in cases.items():
        path = ROOT / f"{GENERATED_PREFIX}negative-{suffix}.yaml"
        path.write_text(common.format(name=name, flags=flags, component=component),
                        encoding="utf-8")
        paths.append((path, expected_error))
    return paths


def _default_esphome_executable() -> str:
    # Prefer the ESPHome console script from the active Python environment.
    # Do NOT resolve sys.executable here: venv Python binaries are commonly
    # symlinks to the system interpreter, and resolving that symlink would make
    # us search next to the system Python instead of inside the venv.
    candidates = [
        Path(sys.executable).with_name("esphome"),
        Path(sys.prefix) / "bin" / "esphome",
    ]
    virtual_env = os.environ.get("VIRTUAL_ENV")
    if virtual_env:
        candidates.append(Path(virtual_env) / "bin" / "esphome")

    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)

    return shutil.which("esphome") or "esphome"


def _run_command(command: list[str], env: dict[str, str]) -> int:
    print("+", " ".join(command), flush=True)
    try:
        return subprocess.run(command, cwd=ROOT, env=env, check=False).returncode
    except FileNotFoundError as err:
        print(f"Command not found: {command[0]} ({err})", file=sys.stderr)
        return 127


def _run_expected_config_failure(command: list[str], env: dict[str, str], expected_error: str) -> bool:
    print("+", " ".join(command), flush=True)
    try:
        proc = subprocess.run(
            command,
            cwd=ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    except FileNotFoundError as err:
        print(f"Command not found: {command[0]} ({err})", file=sys.stderr)
        return False
    sys.stdout.write(proc.stdout)
    sys.stdout.flush()
    if proc.returncode == 0:
        print("Invalid configuration was accepted", file=sys.stderr)
        return False
    if expected_error not in proc.stdout:
        print(f"Expected schema error not found: {expected_error}", file=sys.stderr)
        return False
    return True


def _host_binary_path(variant: str) -> Path:
    name = _host_name(variant)
    return ROOT / ".esphome" / "build" / name / ".pioenvs" / name / "program"


def _run_host_binary(binary: Path, env: dict[str, str], timeout_s: int, variant: str) -> bool:
    if not binary.is_file():
        print(f"Host binary not found after successful compile: {binary}", file=sys.stderr)
        return False

    command = [str(binary)]
    print("+", " ".join(command), flush=True)
    proc = subprocess.Popen(
        command,
        cwd=ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        # Keep the pipe binary and unbuffered. Combining selectors with a
        # buffered TextIOWrapper/readline() can hide already-prefetched lines
        # from select(), causing a false timeout even after the PASS marker was
        # emitted by the child.
        bufsize=0,
    )
    deadline = time.monotonic() + timeout_s
    passed = False
    failed = False
    timed_out = False
    output = bytearray()
    mobile = variant.startswith("mobilesensor-sensirion")
    expected_pass_marker = BRIDGE_PASS_MARKER if mobile else PASS_MARKER
    pass_marker = expected_pass_marker.encode()
    fail_markers = (FAIL_MARKER.encode(), BRIDGE_FAIL_MARKER.encode())

    try:
        assert proc.stdout is not None
        fd = proc.stdout.fileno()
        selector = selectors.DefaultSelector()
        selector.register(fd, selectors.EVENT_READ)
        try:
            while time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                events = selector.select(timeout=min(0.25, remaining))
                if events:
                    chunk = os.read(fd, 65536)
                    if not chunk:
                        break
                    sys.stdout.buffer.write(chunk)
                    sys.stdout.buffer.flush()
                    output.extend(chunk)
                    if any(marker in output for marker in fail_markers):
                        failed = True
                        print("Host binary reported a self-test failure", file=sys.stderr)
                        break
                    if pass_marker in output:
                        passed = True
                        break
                    # Markers are short. Bound retained history while still
                    # allowing a marker split across two reads.
                    if len(output) > 4096:
                        del output[:-4096]
                elif proc.poll() is not None:
                    # Drain anything that became available between poll() and
                    # process exit. os.read() returns b'' at EOF.
                    while True:
                        chunk = os.read(fd, 65536)
                        if not chunk:
                            break
                        sys.stdout.buffer.write(chunk)
                        sys.stdout.buffer.flush()
                        output.extend(chunk)
                    if any(marker in output for marker in fail_markers):
                        failed = True
                    if pass_marker in output:
                        passed = True
                    break

            if not passed and not failed and proc.poll() is None and time.monotonic() >= deadline:
                timed_out = True
        finally:
            selector.close()
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)

    if timed_out:
        print(f"Host binary did not emit {expected_pass_marker!r} within {timeout_s}s", file=sys.stderr)
    return passed and not failed


def _run_smoke(esphome: str, config: Path, variant: str, env: dict[str, str], timeout_s: int) -> bool:
    # `esphome run` is intentionally interactive and keeps a host target alive
    # indefinitely. For automation, compile first and execute the native binary
    # ourselves so we control its lifetime and output capture.
    if _run_command([esphome, "compile", config.name], env) != 0:
        return False
    return _run_host_binary(_host_binary_path(variant), env, timeout_s, variant)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("config", "compile", "run"),
        default="run",
        help="validate, native-compile, or compile+execute each host variant (default: run)",
    )
    parser.add_argument("--timeout", type=int, default=10, help="runtime seconds to wait for each host self-test marker")
    parser.add_argument("--keep-generated", action="store_true")
    parser.add_argument("--esphome", default=_default_esphome_executable())
    args = parser.parse_args()

    toolchain, toolchain_error = _select_native_toolchain()
    if toolchain is None:
        print("Native compiler preflight failed:", file=sys.stderr)
        if toolchain_error:
            print(toolchain_error, file=sys.stderr)
        print(
            "\nHost tests require a native C++20 compiler and standard library. "
            "On macOS the runner can automatically use Homebrew LLVM's libc++ when "
            "the Apple Command Line Tools libc++ lacks the required C++20 concepts.",
            file=sys.stderr,
        )
        return 2
    print(f"Native compiler preflight: {toolchain.description}", flush=True)
    if len(toolchain.compile_flags) > 1:
        print("Native host build flags:", " ".join(toolchain.compile_flags), flush=True)

    failures: list[str] = []
    generated_files: list[Path] = []
    try:
        negative_configs = make_negative_schema_configs(toolchain)
        generated_files.extend(path for path, _ in negative_configs)
        print("\n=== negative schema cases ===", flush=True)
        for negative, expected_error in negative_configs:
            env = os.environ.copy()
            if not _run_expected_config_failure(
                [args.esphome, "config", negative.name], env, expected_error
            ):
                failures.append(f"{negative.name}: expected schema rejection not observed")

        for variant in VARIANTS:
            source = ROOT / variant
            if not source.exists():
                failures.append(f"{variant}: source YAML missing")
                continue

            generated = make_host_config(source, toolchain)
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
                ok = _run_smoke(args.esphome, generated, variant, env, args.timeout)

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
