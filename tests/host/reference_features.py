"""Independent NumPy/SciPy reference for the 51 on-device features.

Usage:
    python reference_features.py test_window.csv features_c.txt

Recomputes every feature with scipy and checks the C implementation
(output of ./run_inference) within a relative tolerance.
"""
import sys

import numpy as np
from scipy import signal, stats

FS = 800.0
N = 1024
BANDS = [(0, 25), (25, 50), (50, 60), (60, 75), (75, 95), (95, 110),
         (110, 125), (125, 160), (160, 220), (220, 250), (250, 350)]


def axis_features(x: np.ndarray) -> list[float]:
    x = x.astype(np.float64) - x.mean()
    rms = np.sqrt(np.mean(x ** 2))
    f, p = signal.welch(x, fs=FS, nperseg=N, window="hann",
                        scaling="density", detrend=False)
    feats = [
        rms,
        x.max() - x.min(),
        np.abs(x).max() / max(rms, 1e-9),
        stats.kurtosis(x, fisher=True, bias=False),
        f[1 + np.argmax(p[1:])],
        np.trapz(p, f) if hasattr(np, "trapz") else np.trapezoid(p, f),
    ]
    df = f[1] - f[0]
    for lo, hi in BANDS:
        feats.append(p[(f >= lo) & (f < hi)].sum() * df)
    return feats


def main() -> int:
    data = np.loadtxt(sys.argv[1], delimiter=",", skiprows=1)
    ref = np.array(sum((axis_features(data[:, a]) for a in range(3)), []))

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
