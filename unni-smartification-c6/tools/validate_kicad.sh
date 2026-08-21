#!/bin/sh
set -eu
KICAD_CLI="${KICAD_CLI:-kicad-cli}"
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p validation spice spice/sim/results

python3 tools/validate_mechanics.py

"$KICAD_CLI" version | tee validation/kicad-version.txt
"$KICAD_CLI" sch export pdf -o validation/schematic.pdf unni-smartification-c6.kicad_sch
"$KICAD_CLI" sch export netlist --format spice -o spice/exported-from-kicad.cir unni-smartification-c6.kicad_sch
"$KICAD_CLI" sch export netlist --format kicadxml -o validation/netlist.xml unni-smartification-c6.kicad_sch
python3 tools/validate_pcb_netmap.py unni-smartification-c6.kicad_pcb validation/netlist.xml
"$KICAD_CLI" sch erc --severity-all -o validation/erc.rpt unni-smartification-c6.kicad_sch || true

# No unsimulatable placeholder symbols may leak into the SPICE netlist.
if grep -E '^[A-Za-z0-9]+ __[A-Za-z0-9_]+' spice/exported-from-kicad.cir; then
  echo 'ERROR: non-simulatable placeholder device found in SPICE netlist' >&2
  exit 2
fi
for token in 'MCP73831' 'TPS63031' 'TPS613222A' 'AO3400A' 'AO3401A' 'USB_HOTPLUG' 'XD1 SYS VBUS' 'V1 ' 'V2 ' 'I1 +3V3' 'I2 SYS'; do
  grep -F "$token" spice/exported-from-kicad.cir >/dev/null || { echo "ERROR: missing SPICE token: $token" >&2; exit 3; }
done
if grep -E '^I3 ' spice/exported-from-kicad.cir >/dev/null; then
  echo 'ERROR: legacy 5V boost proxy I3 must be excluded; TPS613222A model now handles input loading/discharge.' >&2
  exit 3
fi

# Assert critical power and USB nets from KiCad's own exported connectivity.
python3 - <<'PY'
import xml.etree.ElementTree as ET
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
 ('Q3','1'):'Net-(Q3-G)', ('Q3','2'):'GND', ('Q3','3'):'Net-(Q2-G)',
 ('Q4','1'):'VBUS', ('Q4','2'):'GND', ('Q4','3'):'Net-(Q3-G)',
 ('U3','1'):'+5V', ('U3','3'):'GND',
 ('D3','2'):'VBUS', ('D3','1'):'UNNI_AC', ('D4','2'):'+5V', ('D4','1'):'UNNI_AC',
 ('U10','17'):'/ESP32-C6 + Auxiliary/USB_N', ('U10','18'):'/ESP32-C6 + Auxiliary/USB_P',
 ('J1','A7'):'USB_RAW_N', ('J1','A6'):'USB_RAW_P',
 ('U10','12'):'BAT_ADC', ('U10','13'):'USB_ADC', ('U10','5'):'BAT_TEMP_ADC', ('U10','6'):'BACKLIGHT_ADC',
 ('U10','27'):'BOOST_CMD', ('U10','19'):'CHARGE_STAT',
 ('R12','1'):'CHARGE_STAT', ('C9','1'):'UNNI_AC', ('C20','1'):'BAT_ADC', ('C21','1'):'BAT_TEMP_ADC', ('C33','1'):'BACKLIGHT_ADC',
}
bad=[]
for key,want in checks.items():
    got=pin_net.get(key)
    if got!=want: bad.append((key,want,got))
# R13 is passive: orientation is irrelevant, but it must be the 47k VBUS pulldown.
if {pin_net.get(('R13','1')), pin_net.get(('R13','2'))} != {'VBUS','GND'}:
    bad.append((('R13','1/2'),{'VBUS','GND'},{pin_net.get(('R13','1')),pin_net.get(('R13','2'))}))
# MCP73831 PROG must return to GND via 3.3k R3.
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

# In the extracted AppImage environment only missing global library-table warnings are allowed.
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
print('ERC categories:',dict(Counter(cats)))
PY

# Standalone behavioral regression decks.
for deck in spice/sim/ao3400_compare.cir spice/sim/mcp73831_charge.cir spice/sim/3v3_loadstep.cir spice/sim/system_power.cir; do
  echo "== ngspice: $deck =="
  (cd "$(dirname "$deck")" && ../../tools/run_ngspice_shared.py "$(basename "$deck")") >/dev/null
done

# Run a 10-second transient directly from the SPICE netlist exported by KiCad.
python3 tools/make_kicad_transient.py spice/exported-from-kicad.cir validation/kicad-transient.cir
rm -f validation/kicad-system.csv
python3 tools/run_ngspice_shared.py validation/kicad-transient.cir >/dev/null
[ -s validation/kicad-system.csv ] || { echo 'ERROR: KiCad-exported transient did not produce CSV output' >&2; exit 5; }

python3 - <<'PY'
from pathlib import Path
rows=[]
for line in Path('validation/kicad-system.csv').read_text().splitlines():
    p=line.split()
    if len(p)>=12:
        try: rows.append([float(x) for x in p[:12]])
        except ValueError: pass
if not rows: raise SystemExit('no transient rows parsed')
def nearest(t): return min(rows,key=lambda r:abs(r[0]-t))
def vals(t):
    r=nearest(t)
    # time,time, VBUS,VBAT,SYS,3V3,+5V,UNNI,CMD,BOOST_IN,Q2_GATE,Q3_GATE
    return r[2:12]
def require(cond,msg):
    if not cond: raise SystemExit(msg)
def report(t):
    v=vals(t)
    print(f't={t:.2f}s VBUS={v[0]:.3f} VBAT={v[1]:.3f} SYS={v[2]:.3f} 3V3={v[3]:.3f} +5V={v[4]:.3f} UNNI_AC={v[5]:.3f} CMD={v[6]:.3f} BOOST_IN={v[7]:.3f} Q2G={v[8]:.3f} Q3G={v[9]:.3f}')
    return v
# A: battery idle, boost off
v=report(0.10)
require(v[0] < 0.7, f'VBUS too high with USB absent: {v[0]}')
require(3.5 < v[1] < 3.85 and 3.5 < v[2] < 3.85, 'battery/SYS idle operating point invalid')
require(3.15 < v[3] < 3.45, '3V3 not regulated in battery idle')
require(v[4] < 0.2 and v[5] < 0.2 and v[7] < 0.2, 'forced-awake rail unexpectedly active at idle')
# B: forced-awake from battery
v=report(0.70)
require(v[6] > 3.0 and v[7] > 3.3, 'boost command did not enable BOOST_IN')
require(4.8 < v[4] < 5.1 and 4.4 < v[5] < 5.0, 'forced-awake 5V/UNNI_AC invalid')
require(v[9] > 2.5 and v[8] < 0.3, 'Q3/Q2 gate states invalid in forced-awake')
# C: USB inserted while BOOST_CMD deliberately remains asserted; hardware inhibit must win.
v=report(3.10)
require(v[0] > 4.7, 'USB hot-plug not present')
require(v[6] > 3.0, 'BOOST_CMD should remain asserted for inhibit test')
require(v[7] < 0.2 and v[4] < 0.2, 'hardware USB inhibit failed to remove boost input/output')
require(v[8] > 3.4 and v[9] < 0.3, 'Q2/Q3 gates do not show USB hardware inhibit')
require(4.4 < v[5] < 5.0, 'UNNI_AC not maintained from USB after inhibit')
# D: USB removed while command is low: everything 5V-side must fall.
v=report(6.10)
require(v[0] < 0.7 and v[6] < 0.2, 'USB/command phase incorrect after removal')
require(v[7] < 0.2 and v[4] < 0.2 and v[5] < 0.2, '5V path failed to shut down after USB removal')
require(3.15 < v[3] < 3.45, '3V3 not maintained after USB removal')
# E: second forced-awake cycle proves restart after USB removal.
v=report(7.10)
require(v[0] < 0.7 and v[6] > 3.0 and v[7] > 3.3, 'second forced-awake did not start')
require(v[4] > 4.8 and v[5] > 4.4, 'second forced-awake 5V path invalid')
print('KiCad-exported transient phase checks passed.')
PY

echo 'KiCad 10 schematic, connectivity, ERC classification and SPICE validation passed.' | tee validation/summary.txt
