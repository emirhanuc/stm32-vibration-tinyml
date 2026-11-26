"""Check the C feature extractor against the Python reference (training/features.py).

Usage:
    python reference_features.py test_window.csv features_c.txt
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "training"))
from features import window_features  # noqa: E402


def main() -> int:
    data = np.loadtxt(sys.argv[1], delimiter=",", skiprows=1)
    ref = window_features(data)

    c_vals = {}
    for line in open(sys.argv[2]):
        if line.startswith("feature["):
            k, v = line.strip().split("=")
            c_vals[int(k[8:-1])] = float(v)
    c = np.array([c_vals[i] for i in range(len(ref))])

    rel = np.abs(c - ref) / np.maximum(np.abs(ref), 1e-6)
    worst = int(np.argmax(rel))
    print(f"features compared: {len(ref)}")
    print(f"max relative error: {rel.max():.2e} (feature {worst}: C={c[worst]:.6g}, ref={ref[worst]:.6g})")
    ok = rel.max() < 1e-4
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
