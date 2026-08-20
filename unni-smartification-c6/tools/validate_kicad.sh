#!/bin/sh
set -eu
KICAD_CLI="${KICAD_CLI:-kicad-cli}"
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p validation spice spice/sim/results

"$KICAD_CLI" version | tee validation/kicad-version.txt
"$KICAD_CLI" sch export pdf -o validation/schematic.pdf unni-smartification-c6.kicad_sch
"$KICAD_CLI" sch export netlist --format spice -o spice/exported-from-kicad.cir unni-smartification-c6.kicad_sch
"$KICAD_CLI" sch export netlist --format kicadxml -o validation/netlist.xml unni-smartification-c6.kicad_sch
"$KICAD_CLI" sch erc --severity-all -o validation/erc.rpt unni-smartification-c6.kicad_sch || true

# No unsimulatable placeholder symbols may leak into the SPICE netlist.
if grep -E '^[A-Za-z0-9]+ __[A-Za-z0-9_]+' spice/exported-from-kicad.cir; then
  echo 'ERROR: non-simulatable placeholder device found in SPICE netlist' >&2
  exit 2
fi
for token in 'MCP73831' 'TPS63031' 'TPS613222A' 'AO3400A' 'AO3401A' 'XD1 SYS VBUS' 'V1 ' 'V2 ' 'I1 +3V3'; do
  grep -F "$token" spice/exported-from-kicad.cir >/dev/null || { echo "ERROR: missing SPICE token: $token" >&2; exit 3; }
done

# Assert the critical power and USB nets using KiCad's own exported connectivity.
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path
root=ET.parse('validation/netlist.xml').getroot()
pin_net={}
for net in root.find('nets'):
    name=net.attrib['name']
    for n in net.findall('node'):
        pin_net[(n.attrib['ref'],n.attrib['pin'])]=name
checks={
 ('U1','4'):'VBUS', ('U1','3'):'+BATT', ('U1','2'):'GND',
 ('D1','2'):'VBUS', ('D1','1'):'SYS',
 ('Q1','3'):'+BATT', ('Q1','2'):'SYS',
 ('U2','5'):'SYS', ('U2','6'):'SYS', ('U2','8'):'SYS', ('U2','1'):'+3V3', ('U2','10'):'+3V3',
 ('Q2','2'):'+BATT', ('Q2','3'):'/USB + Power/BOOST_IN',
 ('U3','1'):'+5V', ('U3','3'):'GND',
 ('D3','2'):'VBUS', ('D3','1'):'UNNI_AC', ('D4','2'):'+5V', ('D4','1'):'UNNI_AC',
 ('U10','17'):'/ESP32-C6 + Auxiliary/USB_N', ('U10','18'):'/ESP32-C6 + Auxiliary/USB_P',
 ('J1','A7'):'USB_RAW_N', ('J1','A6'):'USB_RAW_P',
 ('U10','12'):'BAT_ADC', ('U10','13'):'USB_ADC', ('U10','5'):'BAT_TEMP_ADC', ('U10','6'):'BACKLIGHT_ADC',
 ('U10','27'):'BOOST_CMD', ('U10','19'):'CHARGE_STAT',
}
bad=[]
for key,want in checks.items():
    got=pin_net.get(key)
    if got!=want: bad.append((key,want,got))
# MCP73831 PROG must be returned to GND via R3=3.3k, not MCU-gated.
prog=pin_net.get(('U1','5'))
if not prog: bad.append((('U1','5'),'program net',None))
if pin_net.get(('R3','1'))!=prog and pin_net.get(('R3','2'))!=prog:
    bad.append((('R3','*'),prog,'not connected to U1 PROG'))
other=pin_net.get(('R3','2')) if pin_net.get(('R3','1'))==prog else pin_net.get(('R3','1'))
if other!='GND': bad.append((('R3','other'),'GND',other))
if bad:
    for x in bad: print('CONNECTIVITY ERROR:',x)
    raise SystemExit(4)
print(f'Connectivity assertions passed: {len(checks)} critical pin/net checks + MCP73831 PROG return.')
PY

# Classify ERC findings: the extracted AppImage environment is allowed to report only
# library-symbol and footprint-link issues caused by absent user-global library tables.
python3 - <<'PY'
import re
from pathlib import Path
s=Path('validation/erc.rpt').read_text(errors='replace')
cats=re.findall(r'^\[([^]]+)\]:',s,re.M)
allowed={'lib_symbol_issues','footprint_link_issues'}
bad=sorted(set(cats)-allowed)
if bad:
    raise SystemExit('Unexpected electrical ERC categories: '+', '.join(bad))
from collections import Counter
c=Counter(cats)
print('ERC categories:',dict(c))
PY

# Run the standalone behavioral tests.
for deck in spice/sim/ao3400_compare.cir spice/sim/mcp73831_charge.cir spice/sim/3v3_loadstep.cir spice/sim/system_power.cir; do
  echo "== ngspice: $deck =="
  (cd "$(dirname "$deck")" && ../../tools/run_ngspice_shared.py "$(basename "$deck")") >/dev/null
done

# Run a transient directly from the SPICE netlist exported from the KiCad hierarchy.
python3 tools/make_kicad_transient.py spice/exported-from-kicad.cir validation/kicad-transient.cir
python3 tools/run_ngspice_shared.py validation/kicad-transient.cir >/dev/null
[ -s validation/kicad-system.csv ] || { echo 'ERROR: KiCad-exported transient did not produce CSV output' >&2; exit 5; }

# Sanity-check key phases from the generated transient CSV.
python3 - <<'PY'
from pathlib import Path
rows=[]
for line in Path('validation/kicad-system.csv').read_text().splitlines():
    p=line.split()
    if len(p)>=9:
        try: rows.append([float(x) for x in p[:9]])
        except ValueError: pass
if not rows: raise SystemExit('no transient rows parsed')
def nearest(t): return min(rows,key=lambda r:abs(r[0]-t))
# columns: scale-time, explicit time, VBUS, VBAT, SYS, 3V3, +5V, UNNI_AC, BOOST_CMD
for t in (1.0,4.0):
    r=nearest(t)
    print('Transient sample',t,r)
# Check that 3V3 is regulated in battery operation and UNNI_AC is energized under forced-awake.
r=nearest(1.0); vbus,vbat,sysv,v33,v5,vac,cmd=r[2:9]
if not (3.15 <= v33 <= 3.45): raise SystemExit(f'3V3 out of expected range at 1s: {v33}')
if not (4.4 <= vac <= 5.1): raise SystemExit(f'UNNI_AC not energized by forced-awake at 1s: {vac}')
r=nearest(4.0); vbus,vbat,sysv,v33,v5,vac,cmd=r[2:9]
if vbus < 4.7: raise SystemExit(f'USB hot-plug not present at 4s: {vbus}')
if not (4.4 <= vac <= 5.1): raise SystemExit(f'UNNI_AC not supplied from USB at 4s: {vac}')
print('Transient sanity checks passed.')
PY

echo 'KiCad 10 schematic, connectivity, ERC classification and SPICE validation passed.' | tee validation/summary.txt
