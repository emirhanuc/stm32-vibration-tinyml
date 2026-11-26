"""Record labelled 1024-sample windows from the board over USB CDC into a CSV.

Usage:
    python collect_dataset.py --port COM9 --label 1.1 --windows 30

Only complete windows with indices 0..1023 in order are written.
"""
import argparse
import csv
import os
import re
from datetime import datetime

import serial

PORT = "COM9"
BAUD_RATE = 115200

WINDOW_SIZE = 1024
TARGET_WINDOWS = 30

LABEL = "normal"

OUTPUT_DIR = "data"

WINDOW_PATTERN = re.compile(
    r"WINDOW\s+(\d+)\s+\|\s+samples=(\d+)\s+\|\s+missed=(\d+)\s+\|\s+dropped=(\d+)"
)


def parse_args():
    global PORT, LABEL, TARGET_WINDOWS, OUTPUT_DIR
    ap = argparse.ArgumentParser(description="Collect labelled vibration windows.")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--label", default=LABEL, help="class label, e.g. 1.1")
    ap.add_argument("--windows", type=int, default=TARGET_WINDOWS)
    ap.add_argument("--out", default=OUTPUT_DIR)
    a = ap.parse_args()
    PORT, LABEL, TARGET_WINDOWS, OUTPUT_DIR = a.port, a.label, a.windows, a.out


def main():
    parse_args()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    filename = os.path.join(
        OUTPUT_DIR,
        f"{LABEL}_{timestamp}.csv"
    )

    print(f"Port aciliyor: {PORT}")

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD_RATE,
        timeout=2
    )

    print(f"{PORT} acildi.")
    print(f"Hedef: {TARGET_WINDOWS} tam pencere")
    print(f"Her pencere: {WINDOW_SIZE} sample")
    print(f"Label: {LABEL}")
    print(f"Dosya: {filename}")
    print()

    accepted_windows = 0

    current_window_id = None
    current_samples = []

    with open(
        filename,
        "w",
        newline="",
        encoding="utf-8"
    ) as csv_file:

        writer = csv.writer(csv_file)

        writer.writerow([
            "session",
            "label",
            "window_id",
            "index",
            "x",
            "y",
            "z",
            "sample_rate_hz"
        ])

        while accepted_windows < TARGET_WINDOWS:

            raw = ser.readline()

            if not raw:
                continue

            line = raw.decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not line:
                continue

            # Yeni pencere başlığı
            match = WINDOW_PATTERN.match(line)

            if match:

                # Önceki pencereyi doğrula
                if current_window_id is not None:

                    if len(current_samples) == WINDOW_SIZE:

                        indices = [
                            sample[0]
                            for sample in current_samples
                        ]

                        expected_indices = list(
                            range(WINDOW_SIZE)
                        )

                        if indices == expected_indices:

                            for index, x, y, z in current_samples:

                                writer.writerow([
                                    timestamp,
                                    LABEL,
                                    current_window_id,
                                    index,
                                    x,
                                    y,
                                    z,
                                    800
                                ])

                            csv_file.flush()

                            accepted_windows += 1

                            print(
                                f"[OK] Window {current_window_id} "
                                f"-> {WINDOW_SIZE} sample "
                                f"({accepted_windows}/{TARGET_WINDOWS})"
                            )

                        else:
                            print(
                                f"[RED] Window {current_window_id}: "
                                f"index sirasi bozuk."
                            )

                    else:
                        print(
                            f"[RED] Window {current_window_id}: "
                            f"{len(current_samples)} sample geldi."
                        )

                # Yeni pencere bilgisi
                current_window_id = int(match.group(1))

                reported_size = int(match.group(2))
                missed = int(match.group(3))
                dropped = int(match.group(4))

                current_samples = []

                if reported_size != WINDOW_SIZE:
                    print(
                        f"[UYARI] STM32 window size: "
                        f"{reported_size}"
                    )

                if missed != 0 or dropped != 0:
                    print(
                        f"[UYARI] Window {current_window_id}: "
                        f"missed={missed}, dropped={dropped}"
                    )

                continue

            # Sample satırı
            parts = line.split(",")

            if len(parts) != 4:
                continue

            if current_window_id is None:
                # Program stream'in ortasında açılmış olabilir.
                continue

            try:
                index = int(parts[0])
                x = int(parts[1])
                y = int(parts[2])
                z = int(parts[3])

            except ValueError:
                print(
                    f"[RED] Bozuk satir: {repr(line)}"
                )
                continue

            current_samples.append(
                (index, x, y, z)
            )

    ser.close()

    print()
    print("====================================")
    print("DATASET TOPLAMA TAMAMLANDI")
    print("====================================")
    print(f"Window       : {accepted_windows}")
    print(
        f"Toplam sample: "
        f"{accepted_windows * WINDOW_SIZE}"
    )
    print(f"Dosya        : {filename}")


if __name__ == "__main__":
    main()