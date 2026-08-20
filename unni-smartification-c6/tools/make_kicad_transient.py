#!/usr/bin/env python3
"""Turn KiCad's exported SPICE netlist into a batch transient validation deck."""
from pathlib import Path
import sys
if len(sys.argv)!=3:
    raise SystemExit(f"usage: {sys.argv[0]} INPUT.cir OUTPUT.cir")
src=Path(sys.argv[1]).read_text()
if not src.rstrip().endswith('.end'):
    raise SystemExit('input does not end in .end')
control=r'''.control
set wr_singlescale
tran 100u 5s
wrdata kicad-system.csv time v(VBUS) v(+BATT) v(SYS) v(+3V3) v(+5V) v(UNNI_AC) v(BOOST_CMD)
.endc
.end
'''
src=src.rstrip()[:-4].rstrip()+"\n"+control
Path(sys.argv[2]).write_text(src)
