#!/bin/sh
set -eu
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
./tools/validate_kicad.sh
python3 tools/run_worst_case_sweep.py
echo 'Full validation including worst-case sweep passed.'
