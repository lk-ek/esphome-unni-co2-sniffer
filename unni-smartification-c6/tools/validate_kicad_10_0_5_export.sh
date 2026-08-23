#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CLI="${KICAD_10_0_5_CLI:-${1:-}}"
if [[ -z "$CLI" ]]; then
  echo "Set KICAD_10_0_5_CLI=/path/to/kicad-cli or pass the CLI path as argument." >&2
  exit 2
fi
python3 "$ROOT/tools/make_kicad_10_0_5_copy.py" --source "$ROOT" --validate --kicad-cli "$CLI"
