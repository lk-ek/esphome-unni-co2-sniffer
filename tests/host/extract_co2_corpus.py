#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Extract CRC-valid CO2 response frames from raw UNNI I2C edge captures."""
from __future__ import annotations
import argparse, csv, io, zipfile
from pathlib import Path


def crc8(a: int, b: int) -> int:
    crc = 0xFF
    for value in (a, b):
        crc ^= value
        for _ in range(8):
            crc = (((crc << 1) ^ 0x31) if (crc & 0x80) else (crc << 1)) & 0xFF
    return crc


def decode_frames(text: str):
    rows = list(csv.DictReader(io.StringIO(text)))
    if not rows:
        return []
    initial = int(rows[0]["initial_state"], 16)
    prev_scl, prev_sda = initial & 1, (initial >> 1) & 1
    active = False
    bits: list[int] = []
    current = None
    frames = []
    for row in rows:
        scl, sda = int(row["scl"]), int(row["sda"])
        if prev_scl == scl == 1 and prev_sda == 1 and sda == 0:
            if current is not None:
                frames.append(current)
            current = {"bytes": [], "acks": []}
            bits = []
            active = True
        elif prev_scl == scl == 1 and prev_sda == 0 and sda == 1 and active:
            if current is not None:
                frames.append(current)
            current = None
            bits = []
            active = False
        if active and prev_scl == 0 and scl == 1:
            bits.append(sda)
            if len(bits) == 9:
                value = 0
                for bit in bits[:8]:
                    value = (value << 1) | bit
                current["bytes"].append(value)
                current["acks"].append(bits[8] == 0)
                bits = []
        prev_scl, prev_sda = scl, sda
    if current is not None:
        frames.append(current)
    return frames


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("archive", type=Path)
    ap.add_argument("output", type=Path, nargs="?", default=Path("tests/host/fixtures/co2_current_260817.csv"))
    args = ap.parse_args()
    retained = []
    with zipfile.ZipFile(args.archive) as zf:
        names = sorted(n for n in zf.namelist() if "/i2c-" in n and n.endswith(".csv"))
        for name in names:
            for frame in decode_frames(zf.read(name).decode("utf-8")):
                raw = frame["bytes"]
                acks = frame["acks"]
                if not raw:
                    continue
                address, direction = raw[0] >> 1, raw[0] & 1
                data = raw[1:]
                if (address == 0x62 and direction == 1 and len(data) >= 3 and len(acks) >= 4 and
                        all(acks[:3]) and not acks[3] and data[2] == crc8(data[0], data[1])):
                    retained.append((name, (data[0] << 8) | data[1], data[0], data[1], data[2]))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as out:
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(("source", "ppm", "msb", "lsb", "crc"))
        writer.writerows(retained)
    print(f"inspected {len(names)} raw I2C CSV captures; retained {len(retained)} CRC-valid CO2 responses")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
