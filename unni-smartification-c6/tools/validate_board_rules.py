#!/usr/bin/env python3
"""Assert project routing rules and the selected JLCPCB-near 4-layer stackup."""
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRO = ROOT / 'unni-smartification-c6.kicad_pro'
PCB = ROOT / 'unni-smartification-c6.kicad_pcb'
p = json.loads(PRO.read_text())
ds = p['board']['design_settings']
r = ds['rules']

def close(a,b,tol=1e-6):
    return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tol)

def die(msg):
    raise SystemExit('BOARD RULE ERROR: '+msg)

want_rules = {
    'min_clearance': 0.15,
    'min_copper_edge_clearance': 0.25,
    'min_hole_clearance': 0.20,
    'min_hole_to_hole': 0.20,
    'min_track_width': 0.15,
    'min_via_annular_width': 0.10,
    'min_via_diameter': 0.50,
    'min_through_hole_diameter': 0.20,
}
for k,v in want_rules.items():
    if not close(r.get(k,-1),v): die(f'{k}={r.get(k)!r}, expected {v}')

want_widths=[0.0,0.15,0.20,0.25,0.30,0.40,0.50,0.60,0.80,1.00,1.20]
if ds.get('track_widths') != want_widths: die('predefined track widths changed')
want_vias=[(0.0,0.0),(0.50,0.20),(0.60,0.30),(0.80,0.40),(1.00,0.50)]
got_vias=[(x['diameter'],x['drill']) for x in ds.get('via_dimensions',[])]
if got_vias != want_vias: die(f'via presets {got_vias!r}')

z=ds['defaults']['zones']
for k,v in {'min_clearance':.25,'min_thickness':.20,'thermal_relief_gap':.30,
            'thermal_relief_spoke_width':.40,'min_island_area':1.0}.items():
    if not close(z.get(k,-1),v): die(f'zone {k}={z.get(k)!r}, expected {v}')

classes={c['name']:c for c in p['net_settings']['classes']}
want_classes={
 'Default':(.20,.20,.60,.30),'Signal':(.20,.20,.60,.30),'Sensitive':(.20,.25,.60,.30),
 'USB':(.20,.20,.60,.30),'Power_3V3':(.40,.20,.60,.30),'Power_5V':(.50,.20,.60,.30),
 'Battery_SYS':(.80,.20,.80,.40),'Switching':(.80,.20,.80,.40),'Ground':(.50,.20,.80,.40),
}
for name,(tw,cl,vd,vdr) in want_classes.items():
    c=classes.get(name)
    if not c: die(f'missing netclass {name}')
    got=(c['track_width'],c['clearance'],c['via_diameter'],c['via_drill'])
    if got!=(tw,cl,vd,vdr): die(f'{name}={got}, expected {(tw,cl,vd,vdr)}')

# Stackup values track JLCPCB's current 4-layer / 1.0 mm JLC3313 construction.
s=PCB.read_text()
def layer_block(name):
    m=re.search(rf'\(layer "{re.escape(name)}"\n(.*?)\n\t\t\t\)',s,re.S)
    if not m: die(f'stackup layer {name} missing')
    return m.group(1)
def val(block,key):
    m=re.search(rf'\({key} ([0-9.]+)\)',block)
    if not m: die(f'{key} missing in stackup block')
    return float(m.group(1))
def textval(block,key):
    m=re.search(rf'\({key} "([^"]+)"\)',block)
    return m.group(1) if m else None

for name,t in [('F.Cu',0.035),('In1.Cu',0.0152),('In2.Cu',0.0152),('B.Cu',0.035)]:
    if not close(val(layer_block(name),'thickness'),t): die(f'{name} copper thickness is not {t} mm')
for name,t,er,material in [
    ('dielectric 1',0.0994,4.1,'JLCPCB 3313 / NP-155F'),
    ('dielectric 2',0.7000,4.53,'Nan Ya NP-155F core'),
    ('dielectric 3',0.0994,4.1,'JLCPCB 3313 / NP-155F'),
]:
    b=layer_block(name)
    if not close(val(b,'thickness'),t): die(f'{name} thickness is not {t} mm')
    if not close(val(b,'epsilon_r'),er): die(f'{name} epsilon_r is not {er}')
    if textval(b,'material') != material: die(f'{name} material changed')

# General target remains 1.0 mm finished nominal board thickness.
m=re.search(r'\(general\s+\(thickness\s+([0-9.]+)\)',s)
if not m or not close(float(m.group(1)),1.0): die('board finished-thickness target is not 1.0 mm')


# Both physical board islands must carry GND on both inner layers.
for required in (
    '(xy 260.3 80.3) (xy 305.7 80.3) (xy 305.7 111.7) (xy 260.3 111.7)',
    '(xy 241.65 78.2) (xy 222.6 78.2) (xy 222.65 143.2) (xy 241.65 143.2)',
):
    found=False
    for zm in re.finditer(r'\(zone\b',s):
        st=zm.start(); depth=0; ins=False; esc=False; end=None
        for i in range(st,len(s)):
            c=s[i]
            if ins:
                if esc: esc=False
                elif c=='\\': esc=True
                elif c=='"': ins=False
            else:
                if c=='"': ins=True
                elif c=='(': depth+=1
                elif c==')':
                    depth-=1
                    if depth==0:
                        end=i+1; break
        b=s[st:end]
        if '(net "GND")' in b and '(layers "In1.Cu" "In2.Cu")' in b and required in b:
            found=True; break
    if not found: die('missing dual-inner-layer GND plane on one board island')

# Ensure the custom manufacturing-rule file contains the via/pad distinction.
dru=(ROOT/'unni-smartification-c6.kicad_dru').read_text()
for needle in ('JLCPCB zone hard minimum','JLCPCB drilled pad hole spacing','hole_to_hole (min 0.45mm)'):
    if needle not in dru: die(f'custom rule missing: {needle}')

print('Board-rule assertions passed: JLCPCB baseline, routing presets, 9 netclasses and JLC3313 1.0 mm near-stackup.')
