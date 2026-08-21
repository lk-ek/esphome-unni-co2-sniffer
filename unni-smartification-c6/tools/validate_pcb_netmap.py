#!/usr/bin/env python3
from pathlib import Path
import re, sys, xml.etree.ElementTree as ET
pcb_path=Path(sys.argv[1] if len(sys.argv)>1 else 'unni-smartification-c6.kicad_pcb')
xml_path=Path(sys.argv[2] if len(sys.argv)>2 else 'validation/netlist.xml')
root=ET.parse(xml_path).getroot()
expected={}
for net in root.findall('.//nets/net'):
    name=net.get('name')
    for node in net.findall('node'):
        expected[(node.get('ref'),node.get('pin'))]=name
s=pcb_path.read_text(errors='replace')

def balanced_end(text,start):
    depth=0; quoted=False; escaped=False
    for i in range(start,len(text)):
        c=text[i]
        if quoted:
            if escaped: escaped=False
            elif c=='\\': escaped=True
            elif c=='"': quoted=False
        else:
            if c=='"': quoted=True
            elif c=='(': depth+=1
            elif c==')':
                depth-=1
                if depth==0:return i+1
    raise SystemExit(f'unbalanced s-expression at offset {start}')

mismatches=[]; checked=0
i=0
while True:
    start=s.find('(footprint ',i)
    if start<0: break
    end=balanced_end(s,start); fp=s[start:end]; i=end
    rm=re.search(r'\(property\s+"Reference"\s+"([^"]+)"',fp)
    if not rm: continue
    ref=rm.group(1); j=0
    while True:
        ps=fp.find('(pad ',j)
        if ps<0: break
        pe=balanced_end(fp,ps); pad=fp[ps:pe]; j=pe
        pm=re.match(r'\(pad\s+"([^"]*)"',pad)
        if not pm or not pm.group(1): continue
        pin=pm.group(1); want=expected.get((ref,pin))
        if want is None: continue
        checked += 1
        nm=re.search(r'\(net\s+"([^"]+)"\)',pad)
        got=nm.group(1) if nm else None
        if got != want: mismatches.append((ref,pin,want,got))
if mismatches:
    for ref,pin,want,got in mismatches:
        print(f'PCB NETMAP ERROR: {ref}.{pin}: schematic={want!r}, pcb={got!r}')
    raise SystemExit(6)
print(f'PCB netmap assertions passed: {checked} schematic-mapped pads match PCB nets.')
