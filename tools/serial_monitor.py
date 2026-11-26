"""Read PM Edge Node output over USB CDC, print predictions, optionally save windows.

The firmware sends, for every 1024-sample window:
    WINDOW <n> | samples=1024 | missed=<m> | dropped=<d>
    <index>,<x>,<y>,<z>            x1024
    AI_RESULT window=<n> class=1.<c> inference_ms=<t>   (or AI_ERROR ...)

Usage:
    python serial_monitor.py --port COM5                 # Windows
    python serial_monitor.py --port /dev/ttyACM0 --save windows.csv --label 1.1
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from dataclasses import dataclass, field
from typing import Iterable, Iterator, TextIO

WINDOW_SIZE = 1024
RE_WINDOW = re.compile(r"WINDOW (\d+) \| samples=(\d+) \| missed=(\d+) \| dropped=(\d+)")
RE_SAMPLE = re.compile(r"^(\d+),(-?\d+),(-?\d+),(-?\d+)$")
RE_RESULT = re.compile(r"AI_RESULT window=(\d+) class=([\d.]+) inference_ms=(\d+)")
RE_ERROR = re.compile(r"AI_ERROR window=(\d+) inference_ms=(\d+)")


@dataclass
class Window:
    number: int
    missed: int
    dropped: int
    samples: list[tuple[int, int, int]] = field(default_factory=list)
    predicted: str | None = None
    inference_ms: int | None = None

    @property
    def complete(self) -> bool:
        return len(self.samples) == WINDOW_SIZE


def parse_stream(lines: Iterable[str]) -> Iterator[Window]:
    """Yield a Window each time an AI_RESULT / AI_ERROR line closes one."""
    current: Window | None = None
    for raw in lines:
        line = raw.strip()
        if not line:
            continue
        if m := RE_WINDOW.search(line):
            current = Window(int(m[1]), int(m[3]), int(m[4]))
        elif (m := RE_SAMPLE.match(line)) and current is not None:
            idx = int(m[1])
            if idx == len(current.samples):          # drop out-of-order rows
                current.samples.append((int(m[2]), int(m[3]), int(m[4])))
        elif (m := RE_RESULT.search(line)) and current is not None:
            current.predicted, current.inference_ms = m[2], int(m[3])
            yield current
            current = None
        elif (m := RE_ERROR.search(line)) and current is not None:
            current.inference_ms = int(m[2])
            yield current
            current = None


def serial_lines(port: str, baud: int) -> Iterator[str]:
    import serial  # pyserial

    with serial.Serial(port, baud, timeout=5) as ser:
        while True:
            yield ser.readline().decode("ascii", errors="replace")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the board (USB CDC)")
    src.add_argument("--file", help="replay a saved text log instead of a port")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC, kept for pyserial")
    ap.add_argument("--save", help="append complete windows to this CSV")
    ap.add_argument("--label", default="", help="class label written to the CSV (e.g. 1.1)")
    args = ap.parse_args(argv)

    lines: Iterable[str] = open(args.file, encoding="utf-8") if args.file else serial_lines(args.port, args.baud)

    out: TextIO | None = None
    writer = None
    if args.save:
        out = open(args.save, "a", newline="", encoding="utf-8")
        writer = csv.writer(out)
        if out.tell() == 0:
            writer.writerow(["window", "index", "x", "y", "z", "label", "predicted"])

    try:
        for w in parse_stream(lines):
            status = w.predicted if w.predicted else "ERROR"
            print(f"window={w.number:5d} samples={len(w.samples):4d} missed={w.missed} "
                  f"dropped={w.dropped} -> class {status} ({w.inference_ms} ms)")
            if writer and w.complete:
                for i, (x, y, z) in enumerate(w.samples):
                    writer.writerow([w.number, i, x, y, z, args.label, w.predicted or ""])
                out.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if out:
            out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
