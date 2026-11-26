"""Feature extraction - the Python twin of firmware/Services/Inference/pm_inference.c.

17 features per axis x 3 axes = 51, on a mean-removed 1024-sample window @ 800 Hz:
    rms, peak_to_peak, crest_factor, kurtosis, dominant_hz, total_power,
    band power in 11 bands (Hz): 0-25, 25-50, 50-60, 60-75, 75-95, 95-110,
    110-125, 125-160, 160-220, 220-250, 250-350

The C implementation is checked against this module in tests/host (max rel. error ~1e-7).
"""
from __future__ import annotations

import numpy as np
from scipy import signal, stats

FS = 800.0
N = 1024
AXES = ("x", "y", "z")
BANDS = [(0, 25), (25, 50), (50, 60), (60, 75), (75, 95), (95, 110),
         (110, 125), (125, 160), (160, 220), (220, 250), (250, 350)]

_PER_AXIS = ["rms", "p2p", "crest", "kurtosis", "dom_hz", "total_power"] + \
            [f"band_{lo}_{hi}" for lo, hi in BANDS]
FEATURE_NAMES = [f"{a}_{n}" for a in AXES for n in _PER_AXIS]


def _trapz(y: np.ndarray, x: np.ndarray) -> float:
    fn = getattr(np, "trapezoid", None) or np.trapz
    return float(fn(y, x))


def axis_features(x: np.ndarray) -> list[float]:
    x = np.asarray(x, dtype=np.float64)
    x = x - x.mean()
    rms = float(np.sqrt(np.mean(x ** 2)))
    f, p = signal.welch(x, fs=FS, nperseg=N, window="hann",
                        scaling="density", detrend=False)
    feats = [
        rms,
        float(x.max() - x.min()),
        float(np.abs(x).max() / max(rms, 1e-9)),
        float(stats.kurtosis(x, fisher=True, bias=False)),
        float(f[1 + np.argmax(p[1:])]),
        _trapz(p, f),
    ]
    df = f[1] - f[0]
    feats += [float(p[(f >= lo) & (f < hi)].sum() * df) for lo, hi in BANDS]
    return feats


def window_features(xyz: np.ndarray) -> np.ndarray:
    """xyz: (1024, 3) raw counts -> (51,) feature vector."""
    if xyz.shape != (N, 3):
        raise ValueError(f"expected ({N}, 3), got {xyz.shape}")
    return np.array(sum((axis_features(xyz[:, a]) for a in range(3)), []), dtype=np.float64)
