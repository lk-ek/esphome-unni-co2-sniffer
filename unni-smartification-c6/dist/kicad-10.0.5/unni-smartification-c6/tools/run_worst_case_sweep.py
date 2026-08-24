#!/usr/bin/env python3
"""Worst-case power sweep derived from the SPICE netlist exported by KiCad.

Sweeps battery OCV and battery internal resistance while deliberately aligning
an ESP radio-current burst with forced-awake 5 V startup.  The hardware
connectivity is not redrawn: each case starts from spice/exported-from-kicad.cir.

Environment:
  NGSPICE_LIB   path to KiCad's bundled libngspice (used by run_ngspice_shared.py)
"""
from __future__ import annotations

import csv
from concurrent.futures import ThreadPoolExecutor, as_completed
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "spice" / "exported-from-kicad.cir"
RUNNER = ROOT / "tools" / "run_ngspice_shared.py"
OUT = ROOT / "validation" / "worst-case"

BATTERY_OCV = (4.2, 3.7, 3.3, 3.0)
BATTERY_R = (0.1, 0.3, 0.5)
STRESS_TIME = 0.70

# Deliberately overlap the ESP radio burst with BOOST_CMD rising at 0.5 s.
# 20 mA baseline, 180 mA burst for 250 ms.
RADIO_PULSE = "PULSE( 0.02 0.18 0.48 1m 1m 0.25 10 )"


def change_once(pattern: str, replacement: str, text: str) -> str:
    text2, n = re.subn(pattern, replacement, text, count=1, flags=re.M)
    if n != 1:
        raise RuntimeError(f"expected exactly one match for {pattern!r}, got {n}")
    return text2


def make_case(base: str, vocv: float, rint: float, csv_name: str) -> str:
    s = base
    s = change_once(
        r"^V2\s+Net-_R300-Pad1_\s+GND\s+DC\s+\S+",
        f"V2 Net-_R300-Pad1_ GND DC {vocv:.3f}", s)
    s = change_once(
        r"^R300\s+Net-_R300-Pad1_\s+\+BATT\s+\S+",
        f"R300 Net-_R300-Pad1_ +BATT {rint:.3f}", s)
    s = change_once(
        r"^I1\s+\+3V3\s+GND\s+PULSE\([^\n]+",
        f"I1 +3V3 GND {RADIO_PULSE} ", s)

    # Replace the simple fixed-current SYS proxy with a conservative power proxy:
    # the converter draws Pout/(eta*Vin), using the actual I1 radio-load waveform.
    # 88% is intentionally conservative for this system-level stress regression.
    s = change_once(
        r"^I2\s+SYS\s+GND\s+PULSE\([^\n]+",
        "B3V3IN SYS GND I={max(I(I1),0)*max(V(+3V3),0)/(0.88*max(V(SYS),1.8))}", s)

    # The normal TPS613222A model contains a 56-ohm equivalent input load, which
    # matches the nominal case around 3.7 V.  At low input voltage that load would
    # under-estimate input current while still maintaining the same 5 V output.
    # Add only the missing current needed to approximate 0.30 W input while +5 V
    # is active.  This makes low-battery cases more conservative without changing
    # the hardware schematic or the normal interactive workbook model.
    marker = "R301 UNNI_AC GND 100"
    extra = (
        marker + "\n"
        "BBOOSTEXTRA /USB_+_Power/BOOST_IN GND "
        "I={min(max((0.30/max(V(/USB_+_Power/BOOST_IN),1.0)-"
        "V(/USB_+_Power/BOOST_IN)/56)*min(max(V(+5V)/4.5,0),1),0),1.0)}"
    )
    if marker not in s:
        raise RuntimeError("UNNI_AC validation load R301 not found")
    s = s.replace(marker, extra, 1)

    s = s.rstrip()
    if not s.endswith(".end"):
        raise RuntimeError("base netlist does not end in .end")
    control = f"""
.control
set wr_singlescale
tran 100u 1.2s
wrdata {csv_name} time v(VBUS) v(+BATT) v(SYS) v(+3V3) v(+5V) v(UNNI_AC) v(BOOST_CMD) v(/USB_+_Power/BOOST_IN) i(V2)
.endc
.end
"""
    return s[:-4] + control


def read_rows(path: Path) -> list[list[float]]:
    rows: list[list[float]] = []
    for line in path.read_text().splitlines():
        p = line.split()
        if len(p) < 11:
            continue
        try:
            rows.append([float(x) for x in p[:11]])
        except ValueError:
            pass
    if not rows:
        raise RuntimeError(f"no data rows in {path}")
    return rows


def nearest(rows: list[list[float]], t: float) -> list[float]:
    return min(rows, key=lambda r: abs(r[0] - t))


def main() -> int:
    if not BASE.exists():
        raise SystemExit(f"missing {BASE}; export the KiCad SPICE netlist first")
    OUT.mkdir(parents=True, exist_ok=True)
    base = BASE.read_text()
    results = []
    env = os.environ.copy()

    def run_case(vocv: float, rint: float) -> dict:
        stem = f"vbat_{vocv:.1f}V_rint_{rint:.1f}ohm"
        csv_name = stem + ".csv"
        deck = OUT / (stem + ".cir")
        deck.write_text(make_case(base, vocv, rint, csv_name))
        p = subprocess.run(
            [sys.executable, str(RUNNER), str(deck)],
            cwd=OUT, env=env, capture_output=True, text=True)
        if p.returncode:
            raise RuntimeError(f"ngspice failed for {stem}\n{p.stdout}\n{p.stderr}")
        csv_path = OUT / csv_name
        rows = read_rows(csv_path)
        r = nearest(rows, STRESS_TIME)
        result = {
            "battery_ocv_v": vocv,
            "battery_r_ohm": rint,
            "stress_time_s": r[0],
            "vbat_terminal_v": r[3],
            "sys_v": r[4],
            "v3v3_v": r[5],
            "boost_5v_v": r[6],
            "unni_ac_v": r[7],
            "boost_in_v": r[9],
            "battery_current_a": -r[10],
        }
        result["rails_pass"] = (
            result["v3v3_v"] >= 3.15
            and result["boost_5v_v"] >= 4.80
            and result["unni_ac_v"] >= 4.40
        )
        # Keep one raw trace/deck for the harshest reference case; the script can
        # regenerate all other raw files at any time.
        if not (abs(vocv-3.0) < 1e-9 and abs(rint-0.5) < 1e-9):
            deck.unlink(missing_ok=True)
            csv_path.unlink(missing_ok=True)
        else:
            keep_csv = OUT / "worst-case-3.0V-0.5ohm.csv"
            keep_deck = OUT / "worst-case-3.0V-0.5ohm.cir"
            csv_path.replace(keep_csv)
            deck.replace(keep_deck)
        return result

    jobs = int(os.environ.get("WORST_CASE_JOBS", "4"))
    cases = [(v, r) for v in BATTERY_OCV for r in BATTERY_R]
    with ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
        futures = {ex.submit(run_case, v, r):(v,r) for v,r in cases}
        for fut in as_completed(futures):
            results.append(fut.result())
    results.sort(key=lambda x: (-x["battery_ocv_v"], x["battery_r_ohm"]))

    fields = list(results[0].keys())
    with (OUT / "worst-case-sweep.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader(); w.writerows(results)

    worst_vbat = min(results, key=lambda x: x["vbat_terminal_v"])
    worst_sys = min(results, key=lambda x: x["sys_v"])
    worst_i = max(results, key=lambda x: x["battery_current_a"])
    failed = [x for x in results if not x["rails_pass"]]

    lines = [
        "# Worst-case battery / forced-awake sweep",
        "",
        "Each case is generated from the **SPICE netlist exported by the actual KiCad hierarchy**.",
        "The ESP radio burst (20 -> 180 mA) is deliberately aligned with forced-awake startup.",
        "For this stress regression only, the 3V3 input proxy is converted to a conservative",
        "constant-power approximation at 88% efficiency, and the 5V boost gets additional",
        "low-voltage input loading so its nominal 56-ohm model does not become optimistic.",
        "",
        "| OCV | Rint | VBAT terminal | SYS | 3V3 | +5V | UNNI_AC | Ibat | Rails |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|:---:|",
    ]
    for x in results:
        lines.append(
            f"| {x['battery_ocv_v']:.1f} V | {x['battery_r_ohm']:.1f} Ω | "
            f"{x['vbat_terminal_v']:.3f} V | {x['sys_v']:.3f} V | {x['v3v3_v']:.3f} V | "
            f"{x['boost_5v_v']:.3f} V | {x['unni_ac_v']:.3f} V | "
            f"{x['battery_current_a']*1000:.0f} mA | {'PASS' if x['rails_pass'] else 'FAIL'} |"
        )
    lines += [
        "",
        f"Lowest steady stressed battery terminal voltage: **{worst_vbat['vbat_terminal_v']:.3f} V** "
        f"at {worst_vbat['battery_ocv_v']:.1f} V OCV / {worst_vbat['battery_r_ohm']:.1f} Ω.",
        f"Lowest steady stressed SYS voltage: **{worst_sys['sys_v']:.3f} V** "
        f"at {worst_sys['battery_ocv_v']:.1f} V OCV / {worst_sys['battery_r_ohm']:.1f} Ω.",
        f"Highest steady modeled battery current: **{worst_i['battery_current_a']*1000:.0f} mA**.",
        "",
        "The 3.0 V OCV cases sag below 3.0 V at the cell terminals. The simulated rails remain",
        "valid, but a real protected cell/BMS may impose a higher practical cutoff. Treat the",
        "3.0 V row as a regulator stress test, not a recommended discharge target.",
        "",
        "These are system-level behavioral-model results. They do not validate switch current",
        "limit, inductor saturation, thermal behavior, loop stability, ripple or EMI.",
    ]
    (OUT / "README.md").write_text("\n".join(lines) + "\n")

    print("Worst-case sweep complete:")
    print(f"  cases: {len(results)}")
    print(f"  rail failures: {len(failed)}")
    print(f"  lowest VBAT terminal: {worst_vbat['vbat_terminal_v']:.3f} V")
    print(f"  lowest SYS: {worst_sys['sys_v']:.3f} V")
    print(f"  highest steady Ibat: {worst_i['battery_current_a']*1000:.0f} mA")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
