#!/usr/bin/env python3
from pathlib import Path
import re
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]

YAGEO_DS = "https://www.yageo.com/upload/media/product/productsearch/datasheet/rchip/PYu-RC_Group_51_RoHS_L_12.pdf"
JST_PH_DS = "https://www.jst-mfg.com/product/pdf/eng/ePH.pdf"
SAMSUNG_MLCC = "https://product.samsungsem.com/mlcc/"
AOS_AO3400_DS = "https://www.aosmd.com/sites/default/files/res/datasheets/AO3400A.pdf"
AOS_AO3401_DS = "https://www.aosmd.com/sites/default/files/res/datasheets/AO3401A.pdf"
MURATA_LQH_DS = "https://search.murata.co.jp/Ceramy/image/img/P02/JELF243A-0021.pdf"
MURATA_DFE_DS = "https://search.murata.co.jp/Ceramy/image/img/P02/JELF243A-0133.pdf"


def tme_search(term):
    return f"https://www.tme.eu/de/katalog/?search={quote(term)}"


def meta(manufacturer, mpn, tme_pn, tme_link, datasheet, value=None, description=None):
    return dict(manufacturer=manufacturer, mpn=mpn, tme_pn=tme_pn, tme_link=tme_link,
                datasheet=datasheet, value=value, description=description)

# exact/special parts
SPECIAL = {
    "U1": meta("Microchip Technology", "MCP73831T-2ACI/OT", "MCP73831T-2ACI/OT",
        "https://www.tme.eu/de/details/mcp73831t-2aci_ot/batterie-u-akku-controller-schaltungen/microchip-technology/",
        "http://ww1.microchip.com/downloads/en/DeviceDoc/20001984g.pdf", "MCP73831"),
    "U2": meta("Texas Instruments", "TPS63031DSKR", "TPS63031DSKR",
        "https://www.tme.eu/en/details/tps63031dskr/voltage-regulators-dc-dc-circuits/texas-instruments/",
        "https://www.ti.com/lit/ds/symlink/tps63030.pdf", "TPS63031"),
    "U3": meta("Texas Instruments", "TPS613222ADBZR", "N/A — not listed at TME",
        tme_search("TPS613222ADBZR"), "https://www.ti.com/lit/ds/symlink/tps61322.pdf", "TPS613222A"),
    "U4": meta("Texas Instruments", "TPD2E009DBZR", "TPD2E009DBZR",
        "https://www.tme.eu/en/details/tpd2e009dbzr/protection-diodes-arrays/texas-instruments/",
        "https://www.ti.com/lit/ds/symlink/tpd2e009.pdf", "TPD2E009DBZR"),
    "U10": meta("Espressif Systems", "ESP32-C6-MINI-1-N4", "ESP32-C6-MINI-1-N4",
        "https://www.tme.eu/de/details/esp32-c6-mini-1-n4/iot-wifi-bluetooth-module/espressif/",
        "https://www.espressif.com/sites/default/files/documentation/esp32-c6-mini-1_mini-1u_datasheet_en.pdf", "ESP32-C6-MINI-1-N4"),
    "Q1": meta("Alpha & Omega Semiconductor", "AO3401A", "AO3401A",
        "https://www.tme.eu/de/details/ao3401a/p-kanal-transistoren-smd/alpha-omega-semiconductor/", AOS_AO3401_DS, "AO3401A"),
    "Q2": meta("Alpha & Omega Semiconductor", "AO3401A", "AO3401A",
        "https://www.tme.eu/de/details/ao3401a/p-kanal-transistoren-smd/alpha-omega-semiconductor/", AOS_AO3401_DS, "AO3401A"),
    "Q3": meta("Alpha & Omega Semiconductor", "AO3400A", "AO3400A",
        "https://www.tme.eu/de/details/ao3400a/n-kanal-transistoren-smd/alpha-omega-semiconductor/", AOS_AO3400_DS, "AO3400A"),
    "Q4": meta("Alpha & Omega Semiconductor", "AO3400A", "AO3400A",
        "https://www.tme.eu/de/details/ao3400a/n-kanal-transistoren-smd/alpha-omega-semiconductor/", AOS_AO3400_DS, "AO3400A"),
    "Q10": meta("Alpha & Omega Semiconductor", "AO3400A", "AO3400A",
        "https://www.tme.eu/de/details/ao3400a/n-kanal-transistoren-smd/alpha-omega-semiconductor/", AOS_AO3400_DS, "AO3400A"),
    "L1": meta("Murata", "LQH3NPN2R2MJRL", "LQH3NPN2R2MJRL",
        "https://www.tme.eu/en/details/lqh3npn2r2mjrl/inductors/murata/", MURATA_LQH_DS, "2.2uH"),
    "L2": meta("Murata", "DFE322520FD-2R2M=P2", "DFE322520FD-2R2M",
        "https://www.tme.eu/en/details/dfe322520fd-2r2m/inductors/murata/dfe322520fd-2r2m-p2/", MURATA_DFE_DS, "2.2uH"),
    "SW1": meta("Jianfu", "TVAF36-N014JW-R", "TVAF36N014JWR",
        "https://www.tme.eu/gb/details/tvaf36n014jwr/microswitches-tact/jianfu/tvaf36-n014jw-r/", tme_search("TVAF36-N014JW-R"), "RESET"),
    "SW2": meta("Jianfu", "TVAF36-N014JW-R", "TVAF36N014JWR",
        "https://www.tme.eu/gb/details/tvaf36n014jwr/microswitches-tact/jianfu/tvaf36-n014jw-r/", tme_search("TVAF36-N014JW-R"), "BOOT"),
    "RT1": meta("SR Passives", "NTCC-10K", "NTCC-10K",
        "https://www.tme.eu/de/details/ntcc-10k/ntc-mess-thermistoren-tht/sr-passives/",
        "https://www.tme.eu/Document/f06e7f4c0fd5fd1c49c3e9fd8ceb0529/NTCC-10K.pdf", "NTCC-10K"),
    "J1": meta("Same Sky", "UJ20-C-H-G-SMT-1-P16-TR", "N/A — not listed at TME",
        tme_search("UJ20-C-H-G-SMT-1-P16-TR"),
        "https://jp.sameskydevices.com/product/resource/uj20-c-h-g-smt-1-p16-tr.pdf", "UJ20-C-H-G-SMT-1-P16-TR"),
}

# JST PH board headers
for ref, pins in {"J2":2, "J3":6, "J5":3, "J6":5, "J10":6}.items():
    mpn = f"B{pins}B-PH-K-S (LF)(SN)"
    tpn = f"B{pins}B-PH-K-S"
    slug = tpn.lower()
    SPECIAL[ref] = meta("JST", mpn, tpn,
        f"https://www.tme.eu/de/details/{slug}/signalsteckverbinder-raster-2-00mm/jst/{slug}-lf-sn/",
        JST_PH_DS, tpn)
SPECIAL["J4"] = meta("JST", "S3B-PH-SM4-TB(LF)(SN)", "S3B-PH-SM4-TB",
    tme_search("S3B-PH-SM4-TB"), JST_PH_DS, "S3B-PH-SM4-TB")

# Schottkys: standardize to currently stocked onsemi SS14 listing at TME.
for ref in ("D1","D2","D3","D4"):
    SPECIAL[ref] = meta("onsemi", "SS14", "SS14-FAI",
        "https://www.tme.eu/en/details/ss14-fai/smd-schottky-diodes/onsemi/ss14/",
        "https://www.onsemi.com/pdf/datasheet/ss12-d.pdf", "SS14" + (" DNP" if ref == "D2" else ""))

# Commodity passives: selected, concrete orderable series so BOM is reproducible.
R0805 = {
 "5.1k":"RC0805FR-075K1L", "1M":"RC0805FR-071ML", "47k":"RC0805FR-0747KL",
 "10k":"RC0805FR-0710KL", "220k":"RC0805FR-07220KL", "150k":"RC0805FR-07150KL",
 "100k":"RC0805FR-07100KL", "22R":"RC0805FR-0722RL", "330k":"RC0805FR-07330KL",
 "10K":"RC0805FR-0710KL",
}
R0603 = {"10k":"RC0603FR-0710KL", "10K":"RC0603FR-0710KL"}
C0603 = {
 "100nF":"CL10B104KB8NNNC", "1uF":"CL10A105KB8NNNC", "220pF":"CL10B221KB8NNNC",
 "22pF DNP":"CL10C220JB8NNNC", "10nF DNP":"CL10B103KB8NNNC", "10uF":"CL10A106MQ8NNNC",
}
C0805 = {
 "4.7uF":"CL21A475KAQNNNE", "10uF":"CL21A106KAYNNNE", "22uF":"CL21A226KPCLRNC",
}

def passive_meta(value, fp):
    if fp.startswith("Resistor_SMD:R_0805") and value in R0805:
        mpn=R0805[value]; tpn=mpn[:-1] if mpn.endswith('L') else mpn
        return meta("Yageo", mpn, tpn, tme_search(mpn), YAGEO_DS)
    if fp.startswith("Resistor_SMD:R_0603") and value in R0603:
        mpn=R0603[value]; tpn=mpn[:-1] if mpn.endswith('L') else mpn
        return meta("Yageo", mpn, tpn, tme_search(mpn), YAGEO_DS)
    if fp.startswith("Capacitor_SMD:C_0603") and value in C0603:
        mpn=C0603[value]
        return meta("Samsung Electro-Mechanics", mpn, mpn, tme_search(mpn), tme_search(mpn))
    if fp.startswith("Capacitor_SMD:C_0805") and value in C0805:
        mpn=C0805[value]
        return meta("Samsung Electro-Mechanics", mpn, mpn, tme_search(mpn), tme_search(mpn))
    return None


def balanced_end(text, start):
    d=0; q=False; esc=False
    for i in range(start,len(text)):
        c=text[i]
        if q:
            if esc: esc=False
            elif c=='\\': esc=True
            elif c=='"': q=False
        else:
            if c=='"': q=True
            elif c=='(': d+=1
            elif c==')':
                d-=1
                if d==0: return i+1
    raise ValueError(start)

def getprop(block,name):
    m=re.search(r'\(property\s+"'+re.escape(name)+r'"\s+"((?:\\.|[^"\\])*)"',block)
    return m.group(1).replace('\\"','"').replace('\\\\','\\') if m else None

def replace_prop(block,name,value):
    # preserve formatting/effects, only replace the string payload
    pat=re.compile(r'(\(property\s+"'+re.escape(name)+r'"\s+")((?:\\.|[^"\\])*)(")')
    esc=value.replace('\\','\\\\').replace('"','\\"')
    if pat.search(block): return pat.sub(lambda m:m.group(1)+esc+m.group(3), block, count=1)
    # add hidden property before Description (or pins) using symbol coordinate
    at=re.search(r'\n\t\t\(at\s+([^\n]+)\)',block)
    pos=at.group(1) if at else '0 0 0'
    prop=(f'\n\t\t(property "{name}" "{esc}"\n\t\t\t(at {pos})\n\t\t\t(hide yes)\n\t\t\t(show_name no)\n\t\t\t(do_not_autoplace no)\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)')
    idx=block.find('\n\t\t(property "Description"')
    if idx<0: idx=block.find('\n\t\t(pin ')
    if idx<0: raise RuntimeError(f'no insertion point for {name}')
    return block[:idx]+prop+block[idx:]


def update_file(path):
    text=path.read_text()
    starts=[m.start()+1 for m in re.finditer(r'\n\t\(symbol\s*\n',text)]
    changed=0; rows=[]
    for start in reversed(starts):
        end=balanced_end(text,start)
        block=text[start:end]
        ref=getprop(block,'Reference'); value=getprop(block,'Value'); fp=getprop(block,'Footprint') or ''
        if not ref or not re.fullmatch(r'[A-Z]+\d+',ref): continue
        # Skip simulation-only sheet by caller; all physical refs are covered by special/passives.
        m=SPECIAL.get(ref) or passive_meta(value,fp)
        if not m:
            rows.append((ref,value,fp,'UNMAPPED'))
            continue
        if m.get('value') is not None: block=replace_prop(block,'Value',m['value'])
        if m.get('description') is not None: block=replace_prop(block,'Description',m['description'])
        block=replace_prop(block,'Datasheet',m['datasheet'])
        block=replace_prop(block,'Manufacturer',m['manufacturer'])
        block=replace_prop(block,'Manufacturer Part Number',m['mpn'])
        block=replace_prop(block,'TME Part Number',m['tme_pn'])
        block=replace_prop(block,'TME Link',m['tme_link'])
        text=text[:start]+block+text[end:]
        changed+=1
        rows.append((ref,getprop(block,'Value'),fp,'OK'))
    path.write_text(text)
    return changed,rows

allrows=[]
for name in ('01_usb_power.kicad_sch','02_esp_aux.kicad_sch'):
    n,rows=update_file(ROOT/name); allrows += [(name,*r) for r in rows]
    print(f'{name}: updated {n} physical symbols')

bad=[r for r in allrows if r[-1] != 'OK']
if bad:
    print('Unmapped physical symbols:')
    for r in bad: print('  ',r)
    raise SystemExit(2)
print(f'All {len(allrows)} physical BOM symbols have component metadata.')
