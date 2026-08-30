#!/usr/bin/env python3
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[1]

def bend(s,i):
 d=0;q=False;e=False
 for j in range(i,len(s)):
  c=s[j]
  if q:
   if e:e=False
   elif c=='\\':e=True
   elif c=='"':q=False
  else:
   if c=='"':q=True
   elif c=='(':d+=1
   elif c==')':
    d-=1
    if d==0:return j+1
 raise ValueError(i)
def prop(b,n):
 m=re.search(r'\(property "'+re.escape(n)+r'" "((?:\\.|[^"\\])*)"',b)
 return m.group(1) if m else None
def setprop(b,n,v):
 esc=v.replace('\\','\\\\').replace('"','\\"')
 p=re.compile(r'(\(property "'+re.escape(n)+r'" ")((?:\\.|[^"\\])*)(")')
 return p.sub(lambda m:m.group(1)+esc+m.group(3),b,count=1)
vals={}
for f in ('01_usb_power.kicad_sch','02_esp_aux.kicad_sch'):
 s=(ROOT/f).read_text()
 for m in re.finditer(r'\n\t\(symbol\s*\n',s):
  i=m.start()+1;b=s[i:bend(s,i)];r=prop(b,'Reference');v=prop(b,'Value')
  if r and re.fullmatch(r'[A-Z]+\d+',r): vals[r]=v
for f in ('unni-smartification-c6.kicad_pcb','experimental/unni-smartification-c6-10.99-constraints.kicad_pcb'):
 p=ROOT/f
 if not p.exists(): continue
 s=p.read_text(); starts=[m.start()+1 for m in re.finditer(r'\n\t\(footprint\s',s)]; n=0
 for i in reversed(starts):
  e=bend(s,i);b=s[i:e];r=prop(b,'Reference')
  if r in vals and prop(b,'Value') != vals[r]:
   b=setprop(b,'Value',vals[r]);s=s[:i]+b+s[e:];n+=1
 p.write_text(s);print(f'{f}: synchronized {n} footprint values')
