"""Quick sanity check: list serial ports, open the board and print the first/last samples of each window."""
import serial
from serial.tools import list_ports

PORT = "COM9"
BAUD_RATE = 115200

print("Mevcut portlar:", flush=True)

for port in list_ports.comports():
    print(f"{port.device} -> {port.description}", flush=True)

print(f"\n{PORT} aciliyor...", flush=True)

ser = serial.Serial(
    port=PORT,
    baudrate=BAUD_RATE,
    timeout=2
)

print(f"{PORT} acildi.", flush=True)
print("STM32 verisi bekleniyor...\n", flush=True)

sample_count = 0
current_window = None

while True:
    raw = ser.readline()

    if not raw:
        continue

    line = raw.decode(
        "utf-8",
        errors="ignore"
    ).strip()

    if not line:
        continue

    if line.startswith("WINDOW"):
        if current_window is not None:
            print(
                f"Window {current_window}: "
                f"{sample_count} sample alindi"
            )

        current_window = line
        sample_count = 0

        print("\n" + line)

        continue

    parts = line.split(",")

    if len(parts) == 4:
        try:
            index = int(parts[0])
            x = int(parts[1])
            y = int(parts[2])
            z = int(parts[3])

            sample_count += 1

            if index < 3 or index >= 1021:
                print(
                    f"{index}: "
                    f"X={x}, Y={y}, Z={z}"
                )

        except ValueError:
            print("Bozuk satir:", repr(line))