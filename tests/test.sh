#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

FULL=0
if [[ "${1:-}" == "--full" ]]; then
  FULL=1
  shift
fi
if [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--full]" >&2
  exit 2
fi

PYTHON="${PYTHON:-python3}"

echo '=== HOST REGRESSION MATRIX ==='
"$PYTHON" tests/host/run_host_tests.py

echo
echo '=== HOST ASAN/UBSAN MATRIX ==='
"$PYTHON" tests/host/run_host_tests.py --sanitize

if (( FULL )); then
  ESPHOME="${ESPHOME:-$($PYTHON - <<'PY'
import os, shutil, sys
from pathlib import Path
for p in (Path(sys.executable).with_name('esphome'), Path(sys.prefix) / 'bin' / 'esphome'):
    if p.is_file() and os.access(p, os.X_OK):
        print(p)
        raise SystemExit
print(shutil.which('esphome') or 'esphome')
PY
)}"
  configs=(
    i2c-sniffer.yaml
    i2c-sniffer-debug.yaml
    i2c-sniffer-no-ble.yaml
    i2c-sniffer-ble-only.yaml
    i2c-sniffer-sht43-probe.yaml
  )
  echo
  echo '=== REAL ESP32-C3 BUILD MATRIX ==='
  for cfg in "${configs[@]}"; do
    echo
    echo "=== $cfg ==="
    "$ESPHOME" compile "$cfg"
  done
fi

echo
if (( FULL )); then
  echo 'PASS: host, ASan/UBSan, and all ESP32-C3 build tests passed'
else
  echo 'PASS: host and ASan/UBSan tests passed'
fi
