#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
KICAD_CLI=${KICAD_CLI:-kicad-cli}
PCB="$ROOT/unni-smartification-c6.kicad_pcb"
SCH="$ROOT/unni-smartification-c6.kicad_sch"
IMG="$ROOT/docs/images"
SCH_IMG="$IMG/schematic"

mkdir -p "$IMG" "$SCH_IMG"

"$KICAD_CLI" pcb render \
  -o "$IMG/pcb-overview-top.png" \
  --width 1800 --height 1100 --quality basic --floor --perspective \
  --rotate 325,0,30 \
  --zoom 0.85 \
  "$PCB"

"$KICAD_CLI" pcb render \
  -o "$IMG/pcb-overview-bottom.png" \
  --width 1800 --height 1100 --quality basic --floor --perspective \
  --side bottom --rotate 325,0,330 \
  --zoom 0.85 \
  "$PCB"

"$KICAD_CLI" pcb render \
  -o "$IMG/pcb-top.png" \
  --width 1800 --height 1100 --quality basic --side top --background opaque \
  "$PCB"

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT INT TERM
"$KICAD_CLI" sch export png -o "$TMP" --dpi 180 "$SCH"

mv "$TMP/unni-smartification-c6.png" "$SCH_IMG/root.png"
mv "$TMP/unni-smartification-c6-USB + Power.png" "$SCH_IMG/usb-power.png"
mv "$TMP/unni-smartification-c6-ESP32-C6 + Auxiliary.png" "$SCH_IMG/esp-aux.png"
mv "$TMP/unni-smartification-c6-Simulation Harness.png" "$SCH_IMG/simulation-harness.png"

printf 'README assets rendered in %s\n' "$IMG"
