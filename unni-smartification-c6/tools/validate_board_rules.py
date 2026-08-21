#!/usr/bin/env python3
"""Assert project routing/netclass/JLCPCB baseline settings."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=json.loads((ROOT/'unni-smartification-c6.kicad_pro').read_text())
ds=p['board']['design_settings']
r=ds['rules']
want_rules={
 'min_clearance':0.15,'min_copper_edge_clearance':0.30,'min_hole_clearance':0.30,
 'min_hole_to_hole':0.45,'min_track_width':0.15,'min_via_annular_width':0.15,
 'min_via_diameter':0.50,'min_through_hole_diameter':0.20,
}
for k,v in want_rules.items():
    if abs(r.get(k,-1)-v)>1e-9: raise SystemExit(f'BOARD RULE ERROR: {k}={r.get(k)!r}, expected {v}')
want_widths=[0.15,0.20,0.25,0.30,0.40,0.50,0.60,0.80,1.00,1.20]
if ds.get('track_widths')!=want_widths: raise SystemExit('BOARD RULE ERROR: predefined track widths changed')
want_vias=[(0.50,0.20),(0.60,0.30),(0.80,0.40),(1.00,0.50)]
got_vias=[(x['diameter'],x['drill']) for x in ds.get('via_dimensions',[])]
if got_vias!=want_vias: raise SystemExit(f'BOARD RULE ERROR: via presets {got_vias!r}')
z=ds['defaults']['zones']
for k,v in {'min_clearance':.25,'min_thickness':.20,'thermal_relief_gap':.30,'thermal_relief_spoke_width':.40,'min_island_area':1.0}.items():
    if abs(z.get(k,-1)-v)>1e-9: raise SystemExit(f'BOARD RULE ERROR: zone {k}={z.get(k)!r}, expected {v}')
classes={c['name']:c for c in p['net_settings']['classes']}
want_classes={
 'Default':(.20,.20,.60,.30),'Signal':(.20,.20,.60,.30),'Sensitive':(.20,.25,.60,.30),
 'USB':(.20,.20,.60,.30),'Power_3V3':(.40,.20,.60,.30),'Power_5V':(.50,.20,.60,.30),
 'Battery_SYS':(.80,.25,.80,.40),'Switching':(.80,.30,.80,.40),'Ground':(.50,.20,.80,.40),
}
for name,(tw,cl,vd,vdr) in want_classes.items():
    c=classes.get(name)
    if not c: raise SystemExit(f'BOARD RULE ERROR: missing netclass {name}')
    got=(c['track_width'],c['clearance'],c['via_diameter'],c['via_drill'])
    if got!=(tw,cl,vd,vdr): raise SystemExit(f'BOARD RULE ERROR: {name}={got}, expected {(tw,cl,vd,vdr)}')
if not (ROOT/'unni-smartification-c6.kicad_dru').exists(): raise SystemExit('BOARD RULE ERROR: custom rules file missing')
print('Board-rule assertions passed: JLCPCB baseline, zone defaults, routing presets and 9 netclasses.')
