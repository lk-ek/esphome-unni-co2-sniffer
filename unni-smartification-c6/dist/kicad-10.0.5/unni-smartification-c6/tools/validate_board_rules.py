#!/usr/bin/env python3
"""Assert project routing rules and the selected PCBWay 4-layer stackup."""
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
    'min_hole_to_hole': 0.30,
    'min_track_width': 0.15,
    'min_via_annular_width': 0.15,
    'min_via_diameter': 0.60,
    'min_through_hole_diameter': 0.30,
}
for k,v in want_rules.items():
    if not close(r.get(k,-1),v): die(f'{k}={r.get(k)!r}, expected {v}')

want_widths=[0.0,0.15,0.20,0.25,0.30,0.40,0.50,0.60,0.80,1.00,1.20]
if ds.get('track_widths') != want_widths: die('predefined track widths changed')
want_vias=[(0.0,0.0),(0.60,0.30),(0.80,0.40),(1.00,0.50)]
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

# Stackup values track PCBWay's current regular 4-layer / 1.0 mm / 1 oz-inner construction.
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

for name,t in [('F.Cu',0.035),('In1.Cu',0.035),('In2.Cu',0.035),('B.Cu',0.035)]:
    if not close(val(layer_block(name),'thickness'),t): die(f'{name} copper thickness is not {t} mm')
for name,t,er,material in [
    ('dielectric 1',0.1855,4.74,'PCBWay 7628 RC46% prepreg'),
    ('dielectric 2',0.4300,4.6,'PCBWay FR-4 core'),
    ('dielectric 3',0.1855,4.74,'PCBWay 7628 RC46% prepreg'),
]:
    b=layer_block(name)
    if not close(val(b,'thickness'),t): die(f'{name} thickness is not {t} mm')
    if not close(val(b,'epsilon_r'),er): die(f'{name} epsilon_r is not {er}')
    if textval(b,'material') != material: die(f'{name} material changed')

# General target remains 1.0 mm finished nominal board thickness.
m=re.search(r'\(general\s+\(thickness\s+([0-9.]+)\)',s)
if not m or not close(float(m.group(1)),1.0): die('board finished-thickness target is not 1.0 mm')


# Both physical board islands must carry a GND zone on both inner layers.
# Detect this geometrically rather than hard-coding absolute sheet coordinates so
# the two-board layout can be translated on the drawing sheet.
def balanced_end(text,start):
    depth=0; ins=False; esc=False
    for i in range(start,len(text)):
        c=text[i]
        if ins:
            if esc: esc=False
            elif c=='\\': esc=True
            elif c=='"': ins=False
        else:
            if c=='"': ins=True
            elif c=='(': depth+=1
            elif c==')':
                depth-=1
                if depth==0: return i+1
    die('unterminated S-expression')

edge_lines=[]
for gm in re.finditer(r'\(gr_line\b',s):
    b=s[gm.start():balanced_end(s,gm.start())]
    if '(layer "Edge.Cuts")' not in b: continue
    m=re.search(r'\(start ([0-9.-]+) ([0-9.-]+)\).*?\(end ([0-9.-]+) ([0-9.-]+)\)',b,re.S)
    if m: edge_lines.append(tuple(map(float,m.groups())))
if len(edge_lines)!=8: die(f'expected 8 Edge.Cuts lines, got {len(edge_lines)}')
# Rectangles are axis aligned; unique x/y extrema split naturally into the two islands.
pts=[(l[0],l[1]) for l in edge_lines]+[(l[2],l[3]) for l in edge_lines]
# connected components by shared endpoints
remaining=list(edge_lines); rects=[]
while remaining:
    comp=[remaining.pop()]; changed=True
    while changed:
        changed=False
        cpts={(l[0],l[1]) for l in comp}|{(l[2],l[3]) for l in comp}
        rest=[]
        for ln in remaining:
            if (ln[0],ln[1]) in cpts or (ln[2],ln[3]) in cpts:
                comp.append(ln); changed=True
            else: rest.append(ln)
        remaining=rest
    xs=[v for l in comp for v in (l[0],l[2])]; ys=[v for l in comp for v in (l[1],l[3])]
    rects.append((min(xs),min(ys),max(xs),max(ys)))

zones=[]
for zm in re.finditer(r'\(zone\b',s):
    b=s[zm.start():balanced_end(s,zm.start())]
    if '(net "GND")' not in b or '(layers "In1.Cu" "In2.Cu")' not in b: continue
    xy=[tuple(map(float,m)) for m in re.findall(r'\(xy ([0-9.-]+) ([0-9.-]+)\)',b)]
    if xy: zones.append((min(x for x,y in xy),min(y for x,y in xy),max(x for x,y in xy),max(y for x,y in xy)))
for r in rects:
    cx=(r[0]+r[2])/2; cy=(r[1]+r[3])/2
    if not any(z[0]-1e-6<=cx<=z[2]+1e-6 and z[1]-1e-6<=cy<=z[3]+1e-6 for z in zones):
        die(f'missing dual-inner-layer GND plane on board island {r}')

# Ensure the custom manufacturing-rule file contains the via/pad distinction.
dru=(ROOT/'unni-smartification-c6.kicad_dru').read_text()
for needle in ('PCBWay zone hard minimum','PCBWay drilled pad hole spacing','hole_to_hole (min 0.45mm)'):
    if needle not in dru: die(f'custom rule missing: {needle}')

print('Board-rule assertions passed: PCBWay baseline, routing presets, 9 netclasses and 1.0 mm 1/1 oz stackup.')
