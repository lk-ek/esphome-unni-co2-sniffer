#!/usr/bin/env python3
"""Run an ngspice control-deck through KiCad's bundled libngspice.

Usage: run_ngspice_shared.py FILE.cir
Set NGSPICE_LIB to override the bundled-library path.
"""
from __future__ import annotations
import ctypes, os, pathlib, sys

if len(sys.argv) != 2:
    raise SystemExit(f"usage: {sys.argv[0]} FILE.cir")
netlist = pathlib.Path(sys.argv[1]).resolve()
if not netlist.exists():
    raise SystemExit(f"missing netlist: {netlist}")
libpath = os.environ.get("NGSPICE_LIB", "/mnt/data/kicad-app/AppDir/shared/lib/libngspice.so.0.0.14")
lib = ctypes.CDLL(libpath)

SENDCHAR = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
SENDSTAT = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
EXITCB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_int, ctypes.c_void_p)
SENDDATA = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p)
SENDINIT = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
BGCB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_bool, ctypes.c_int, ctypes.c_void_p)
errors=[]
@SENDCHAR
def sendchar(msg, ident, user):
    if msg:
        text=msg.decode(errors="replace")
        print(text)
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

lib.ngSpice_Init.argtypes=[SENDCHAR,SENDSTAT,EXITCB,SENDDATA,SENDINIT,BGCB,ctypes.c_void_p]
lib.ngSpice_Init.restype=ctypes.c_int
lib.ngSpice_Command.argtypes=[ctypes.c_char_p]
lib.ngSpice_Command.restype=ctypes.c_int
if lib.ngSpice_Init(sendchar,sendstat,controlled_exit,senddata,sendinit,bg,None) != 0:
    raise SystemExit("ngSpice_Init failed")
os.chdir(netlist.parent)
rc=lib.ngSpice_Command(f"source {netlist}".encode())
if rc != 0 or errors:
    raise SystemExit(f"ngspice failed (rc={rc}); stderr messages={len(errors)}")
