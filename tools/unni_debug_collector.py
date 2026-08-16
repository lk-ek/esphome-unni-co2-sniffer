#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Receive Unni CO2 sniffer UDP debug captures and archive them on macOS/Linux."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import select
import socket
import struct
import sys
import termios
import tty
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

MAGIC = b"UND1"
VERSION = 1
HEADER = struct.Struct("<4sBBHIHHHH")
TYPE_I2C_LA02 = 1
TYPE_RTRH_RAW = 2
TYPE_RTRH_TIMING = 3
TIMING = struct.Struct("<IBBBB I ffffff HHHH BBBB")

TYPE_NAMES = {
    TYPE_I2C_LA02: "i2c",
    TYPE_RTRH_RAW: "rtrh",
    TYPE_RTRH_TIMING: "timing",
}


@dataclass
class PendingCapture:
    packet_count: int
    flags: int
    first_seen: dt.datetime
    packets: Dict[int, bytes] = field(default_factory=dict)

    def complete(self) -> bool:
        return len(self.packets) == self.packet_count

    def payload(self) -> bytes:
        return b"".join(self.packets[i] for i in range(self.packet_count))


def stamp() -> str:
    return dt.datetime.now().astimezone().strftime("%Y%m%d-%H%M%S.%f")[:-3]


def day_dir(root: Path) -> Path:
    target = root / dt.date.today().isoformat()
    target.mkdir(parents=True, exist_ok=True)
    return target


def write_i2c(root: Path, capture_id: int, data: bytes) -> Tuple[Path, Path]:
    target = day_dir(root)
    base = f"i2c-{capture_id:08d}-{stamp()}"
    la_path = target / f"{base}.la"
    la_path.write_bytes(data)

    if len(data) < 10 or data[:4] != b"LA02":
        raise ValueError("invalid LA02 payload")
    count = struct.unpack_from("<I", data, 4)[0]
    overflow = data[8]
    initial = data[9]
    expected = 10 + count * 5
    if len(data) != expected:
        raise ValueError(f"LA02 length mismatch: got {len(data)}, expected {expected}")

    csv_path = target / f"{base}.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sequence", "t_us", "scl", "sda", "state", "initial_state", "overflow"])
        off = 10
        for _ in range(count):
            t_us = struct.unpack_from("<I", data, off)[0]
            value = data[off + 4]
            off += 5
            w.writerow([
                capture_id,
                t_us,
                1 if value & 0x01 else 0,
                1 if value & 0x02 else 0,
                f"0x{value:02X}",
                f"0x{initial:02X}",
                1 if overflow else 0,
            ])
    return la_path, csv_path


def write_rtrh(root: Path, capture_id: int, data: bytes) -> Path:
    if len(data) < 4:
        raise ValueError("RT/RH payload too short")
    count, overflow, _reserved = struct.unpack_from("<HBB", data, 0)
    expected = 4 + count * 7
    if len(data) != expected:
        raise ValueError(f"RT/RH length mismatch: got {len(data)}, expected {expected}")

    target = day_dir(root)
    path = target / f"rtrh-{capture_id:08d}-{stamp()}.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        # Keep historical gpio10/gpio13 column names out of the new transport.
        w.writerow(["sequence", "t_us", "edge_no", "rt_gpio3", "rh_gpio4", "state", "overflow"])
        off = 4
        for _ in range(count):
            t_us, edge_no, value = struct.unpack_from("<IHB", data, off)
            off += 7
            w.writerow([
                capture_id,
                t_us,
                edge_no,
                1 if value & 0x01 else 0,
                1 if value & 0x08 else 0,
                f"0x{value:02X}",
                1 if overflow else 0,
            ])
    return path


def write_timing(root: Path, data: bytes) -> Tuple[Path, List[str], List[object]]:
    if len(data) != TIMING.size:
        raise ValueError(f"timing payload length {len(data)} != {TIMING.size}")
    (
        sequence, valid, reject_reason, rh_samples, flags, rh_seen,
        quality, ref_us, rt_us, rh_us, temp_c, humidity,
        rh_min, rh_p25, rh_p75, rh_max, near220, near440, other, _reserved,
    ) = TIMING.unpack(data)

    target = day_dir(root)
    path = target / "rtrh_timing.csv"
    new = not path.exists()
    header = [
        "received_at", "sequence", "valid", "reject_reason", "quality_percent",
        "ref_period_us", "rt_period_us", "rh_state_us", "temperature_c",
        "humidity_percent", "rh_state_samples", "rh_state_seen", "rh_min_us",
        "rh_p25_us", "rh_p75_us", "rh_max_us", "near_220", "near_440", "other",
        "thermal_transient", "temperature_extrapolation", "humidity_extrapolation",
        "calibration_extrapolation",
    ]
    row: List[object] = [
        dt.datetime.now().astimezone().isoformat(timespec="milliseconds"),
        sequence, valid, reject_reason, f"{quality:.3f}", f"{ref_us:.3f}",
        f"{rt_us:.3f}", f"{rh_us:.3f}", f"{temp_c:.3f}", f"{humidity:.3f}",
        rh_samples, rh_seen, rh_min, rh_p25, rh_p75, rh_max, near220, near440, other,
        1 if flags & 0x01 else 0, 1 if flags & 0x02 else 0,
        1 if flags & 0x04 else 0, 1 if flags & 0x08 else 0,
    ]
    with path.open("a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(header)
        w.writerow(row)
    return path, header, row


def expire_old(pending: Dict[Tuple[str, int, int], PendingCapture], timeout_s: float) -> None:
    now = dt.datetime.now().astimezone()
    for key, cap in list(pending.items()):
        age = (now - cap.first_seen).total_seconds()
        if age < timeout_s:
            continue
        missing = [str(i + 1) for i in range(cap.packet_count) if i not in cap.packets]
        src, ptype, capture_id = key
        print(f"[{now:%H:%M:%S}] INCOMPLETE {TYPE_NAMES.get(ptype, ptype)} #{capture_id} "
              f"from {src}: missing packet(s) {','.join(missing)}")
        del pending[key]



class KeypressMode:
    def __init__(self) -> None:
        self.fd: Optional[int] = None
        self.old_termios = None

    def __enter__(self) -> "KeypressMode":
        if sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.old_termios = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.fd is not None and self.old_termios is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_termios)

    def waitables(self, sock: socket.socket):
        if self.fd is None:
            return [sock]
        return [sock, sys.stdin]

    def consume_key(self) -> bool:
        if self.fd is None:
            return False
        data = sys.stdin.read(1)
        return bool(data)


def archive_batch(root: Path, files: List[Path], timing_header: Optional[List[str]],
                  timing_rows: List[List[object]], batch_started: dt.datetime) -> Optional[Path]:
    # A keypress marks a boundary between test phases. Only files completed since
    # the previous boundary are included. The append-only timing file is represented
    # by a batch-local CSV so older timing rows do not leak into later archives.
    existing = []
    seen = set()
    for path in files:
        try:
            resolved = path.resolve()
        except OSError:
            continue
        if resolved in seen or not path.exists():
            continue
        seen.add(resolved)
        existing.append(path)

    if not existing and not timing_rows:
        print(f"[{dt.datetime.now().astimezone():%H:%M:%S}] no completed captures since last archive boundary")
        return None

    end = dt.datetime.now().astimezone()
    name = f"unni-captures-{batch_started:%Y%m%d-%H%M%S}-{end:%H%M%S}.zip"
    archive = root / name
    suffix = 1
    while archive.exists():
        archive = root / f"{Path(name).stem}-{suffix}.zip"
        suffix += 1

    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for path in existing:
            try:
                arcname = path.relative_to(root)
            except ValueError:
                arcname = Path(path.name)
            zf.write(path, arcname)

        if timing_rows and timing_header is not None:
            import io
            buf = io.StringIO(newline="")
            w = csv.writer(buf)
            w.writerow(timing_header)
            w.writerows(timing_rows)
            zf.writestr("rtrh_timing.csv", buf.getvalue())

        manifest = (
            f"batch_started={batch_started.isoformat(timespec='milliseconds')}\n"
            f"batch_archived={end.isoformat(timespec='milliseconds')}\n"
            f"capture_files={len(existing)}\n"
            f"timing_records={len(timing_rows)}\n"
        )
        zf.writestr("capture_batch.txt", manifest)

    print(f"[{end:%H:%M:%S}] archived captures since last keypress -> {archive}")
    return archive

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen", default="10.0.42.149", help="local IPv4 address (default: 10.0.42.149)")
    parser.add_argument("--port", type=int, default=45678)
    parser.add_argument("--output", type=Path, default=Path("unni-debug"))
    parser.add_argument("--incomplete-timeout", type=float, default=5.0)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen, args.port))
    sock.setblocking(False)
    pending: Dict[Tuple[str, int, int], PendingCapture] = {}
    completed_timing: Dict[Tuple[str, int, int], dt.datetime] = {}
    batch_files: List[Path] = []
    batch_timing_header: Optional[List[str]] = None
    batch_timing_rows: List[List[object]] = []
    batch_started = dt.datetime.now().astimezone()
    print(f"Listening on udp://{args.listen}:{args.port} -> {args.output.resolve()}")
    print("Press any key to archive all completed captures since start/last keypress; Ctrl-C stops.")

    with KeypressMode() as keys:
      try:
       while True:
        ready, _, _ = select.select(keys.waitables(sock), [], [], 1.0)
        if sys.stdin in ready:
            if keys.consume_key():
                archive_batch(args.output, batch_files, batch_timing_header, batch_timing_rows, batch_started)
                batch_files.clear()
                batch_timing_rows.clear()
                batch_timing_header = None
                batch_started = dt.datetime.now().astimezone()
            ready = [item for item in ready if item is not sys.stdin]

        if sock not in ready:
            expire_old(pending, args.incomplete_timeout)
            continue

        try:
            datagram, addr = sock.recvfrom(2048)
        except BlockingIOError:
            continue

        now = dt.datetime.now().astimezone()
        if len(datagram) < HEADER.size:
            print(f"[{now:%H:%M:%S}] short datagram from {addr[0]}")
            continue
        magic, version, ptype, flags, capture_id, index, count, payload_len, _ = HEADER.unpack_from(datagram)
        payload = datagram[HEADER.size:]
        if magic != MAGIC or version != VERSION or payload_len != len(payload) or count == 0 or index >= count:
            print(f"[{now:%H:%M:%S}] invalid datagram from {addr[0]}")
            continue

        key = (addr[0], ptype, capture_id)
        if ptype == TYPE_RTRH_TIMING:
            # Firmware intentionally sends every timing record twice, 20 ms apart.
            # Keep only the first completed copy while still allowing a later
            # capture-id reuse after the dedupe window expires.
            cutoff = now - dt.timedelta(seconds=60)
            completed_timing = {k: t for k, t in completed_timing.items() if t >= cutoff}
            if key in completed_timing:
                continue
        cap = pending.get(key)
        if cap is None or cap.packet_count != count:
            cap = PendingCapture(count, flags, now)
            pending[key] = cap
        cap.packets[index] = payload
        if not cap.complete():
            continue

        data = cap.payload()
        del pending[key]
        try:
            if ptype == TYPE_I2C_LA02:
                la_path, path = write_i2c(args.output, capture_id, data)
                batch_files.extend([la_path, path])
            elif ptype == TYPE_RTRH_RAW:
                path = write_rtrh(args.output, capture_id, data)
                batch_files.append(path)
            elif ptype == TYPE_RTRH_TIMING:
                path, timing_header, timing_row = write_timing(args.output, data)
                batch_timing_header = timing_header
                batch_timing_rows.append(timing_row)
                completed_timing[key] = now
            else:
                print(f"[{now:%H:%M:%S}] unknown packet type {ptype} from {addr[0]}")
                continue
            print(f"[{now:%H:%M:%S}] {TYPE_NAMES.get(ptype, ptype)} #{capture_id} "
                  f"complete ({count}/{count} packets) -> {path}")
        except Exception as exc:
            print(f"[{now:%H:%M:%S}] failed to decode {TYPE_NAMES.get(ptype, ptype)} #{capture_id}: {exc}")

        expire_old(pending, args.incomplete_timeout)
      except KeyboardInterrupt:
        print("\nStopped.")
        return


if __name__ == "__main__":
    main()
