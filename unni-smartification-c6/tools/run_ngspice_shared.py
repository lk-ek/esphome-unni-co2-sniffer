#!/usr/bin/env python3
"""Run an ngspice control deck portably.

Resolution order:
  1. NGSPICE_LIB (explicit shared-library path)
  2. KiCad/Homebrew/system libngspice discovery
  3. NGSPICE_BIN or an `ngspice` executable in PATH

This keeps the project self-test usable on the Linux KiCad AppImage test
container as well as a normal macOS KiCad installation.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import glob
import os
from pathlib import Path
import shutil
import subprocess
import sys

if len(sys.argv) != 2:
    raise SystemExit(f"usage: {sys.argv[0]} FILE.cir")
netlist = Path(sys.argv[1]).resolve()
if not netlist.exists():
    raise SystemExit(f"missing netlist: {netlist}")


def existing(paths):
    seen = set()
    for p in paths:
        if not p:
            continue
        p = os.path.realpath(os.path.expanduser(p))
        if p not in seen and os.path.isfile(p):
            seen.add(p)
            yield p


def discover_libngspice() -> str | None:
    explicit = os.environ.get("NGSPICE_LIB")
    if explicit:
        if not os.path.isfile(explicit):
            raise SystemExit(f"NGSPICE_LIB does not exist: {explicit}")
        return os.path.realpath(explicit)

    candidates: list[str] = []

    # Derive likely Frameworks/lib locations from kicad-cli if available.
    kicad_cli = os.environ.get("KICAD_CLI") or shutil.which("kicad-cli")
    if kicad_cli:
        kc = Path(kicad_cli).resolve()
        # macOS: .../KiCad.app/Contents/MacOS/kicad-cli
        for parent in [kc.parent, *kc.parents]:
            if parent.name == "Contents":
                fw = parent / "Frameworks"
                candidates += [
                    str(fw / "libngspice.0.dylib"),
                    str(fw / "libngspice.dylib"),
                ]
                candidates += glob.glob(str(fw / "libngspice*.dylib"))
                break

    # Standard KiCad macOS app locations (stable and nightly layouts).
    for app in [
        "/Applications/KiCad/KiCad.app/Contents",
        "/Applications/KiCad/KiCad Nightly.app/Contents",
    ]:
        candidates += glob.glob(app + "/Frameworks/libngspice*.dylib")
        candidates += glob.glob(app + "/MacOS/libngspice*.dylib")
        candidates += glob.glob(app + "/PlugIns/libngspice*.dylib")

    # Homebrew and common Unix locations.
    candidates += glob.glob("/opt/homebrew/lib/libngspice*.dylib")
    candidates += glob.glob("/usr/local/lib/libngspice*.dylib")
    candidates += glob.glob("/opt/homebrew/Cellar/ngspice/*/lib/libngspice*.dylib")
    candidates += glob.glob("/usr/local/Cellar/ngspice/*/lib/libngspice*.dylib")
    candidates += glob.glob("/usr/lib*/libngspice.so*")
    candidates += glob.glob("/usr/local/lib*/libngspice.so*")

    # Project author's extracted Linux KiCad AppImage environment, if present.
    candidates += glob.glob("/mnt/data/kicad-app/AppDir/shared/lib/libngspice.so*")
    candidates += glob.glob("/mnt/data/kicad-env/**/libngspice.so*", recursive=True)

    # Let the platform loader suggest a library name/path too.
    found = ctypes.util.find_library("ngspice")
    if found:
        if os.path.isabs(found):
            candidates.append(found)
        else:
            # ctypes can load a soname directly even if it is not a filesystem path.
            try:
                ctypes.CDLL(found)
                return found
            except OSError:
                pass

    return next(existing(candidates), None)


def run_via_shared(libpath: str) -> None:
    lib = ctypes.CDLL(libpath)

    SENDCHAR = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
    SENDSTAT = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
    EXITCB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_int, ctypes.c_void_p)
    SENDDATA = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p)
    SENDINIT = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
    BGCB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_bool, ctypes.c_int, ctypes.c_void_p)
    errors: list[str] = []

    @SENDCHAR
    def sendchar(msg, ident, user):
        if msg:
            text = msg.decode(errors="replace")
            print(text)
            # ngspice prefixes actual simulator errors with stderr Error: in shared mode.
            if text.startswith("stderr Error:"):
                errors.append(text)
        return 0

    @SENDSTAT
    def sendstat(msg, ident, user): return 0
    @EXITCB
    def controlled_exit(status, immediate, quitexit, ident, user): return 0
    @SENDDATA
    def senddata(data, nvec, ident, user): return 0
    @SENDINIT
    def sendinit(data, ident, user): return 0
    @BGCB
    def bg(running, ident, user): return 0

    lib.ngSpice_Init.argtypes = [SENDCHAR, SENDSTAT, EXITCB, SENDDATA, SENDINIT, BGCB, ctypes.c_void_p]
    lib.ngSpice_Init.restype = ctypes.c_int
    lib.ngSpice_Command.argtypes = [ctypes.c_char_p]
    lib.ngSpice_Command.restype = ctypes.c_int
    if lib.ngSpice_Init(sendchar, sendstat, controlled_exit, senddata, sendinit, bg, None) != 0:
        raise SystemExit("ngSpice_Init failed")
    os.chdir(netlist.parent)
    rc = lib.ngSpice_Command(f"source {netlist.name}".encode())
    if rc != 0 or errors:
        raise SystemExit(f"ngspice failed (rc={rc}); stderr messages={len(errors)}")


def run_via_executable(exe: str) -> None:
    # .control sections execute in batch mode; use the deck directory so relative
    # .include and wrdata paths behave exactly as in shared-library mode.
    p = subprocess.run(
        [exe, "-b", netlist.name],
        cwd=netlist.parent,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if p.stdout:
        print(p.stdout, end="")
    if p.returncode != 0:
        raise SystemExit(f"ngspice executable failed (rc={p.returncode})")
    # Some ngspice builds return zero while still printing a fatal parse/runtime error.
    lowered = p.stdout.lower()
    fatal_markers = ("fatal error", "error: no such", "unknown device type", "could not find include file")
    if any(x in lowered for x in fatal_markers):
        raise SystemExit("ngspice executable reported a fatal error")


libpath = discover_libngspice()
if libpath:
    run_via_shared(libpath)
else:
    exe = os.environ.get("NGSPICE_BIN") or shutil.which("ngspice")
    if exe:
        run_via_executable(exe)
    else:
        raise SystemExit(
            "Could not locate libngspice or ngspice.\n"
            "Set NGSPICE_LIB=/path/to/libngspice (preferred), or NGSPICE_BIN=/path/to/ngspice.\n"
            "On macOS with Homebrew, `brew install ngspice` is a fallback; a normal KiCad app may also bundle libngspice under KiCad.app/Contents/Frameworks."
        )
