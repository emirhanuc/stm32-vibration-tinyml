"""Train the fan-vibration classifiers and export the deployed model as a C header.

    python training/train.py             # train, evaluate, export to training/output/
    python training/train.py --deploy    # also overwrite the firmware model header

Data: data/raw/class_<label>.csv, one file per fan level, recorded with
tools/collect_dataset.py. The class is taken from the file name (the `label`
column inside the CSV is not used).

Protocol (same as the original pilot, seed 42):
  - 75 / 25 stratified window holdout -> models are fitted on the 75 % split
  - 5-fold stratified CV reported as a second, less split-dependent estimate
  - Random Forest is exported for the MCU; logistic regression (scaler folded
    into the weights) is exported as a baseline; an MLP is compared only.

WARNING: one recording session per class. Window-level splits are NOT
independent-session validation; scores are exploratory.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (accuracy_score, balanced_accuracy_score, classification_report,
                             confusion_matrix, f1_score)
from sklearn.model_selection import StratifiedKFold, cross_val_score, train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

sys.path.insert(0, str(Path(__file__).resolve().parent))
from features import FEATURE_NAMES, N, window_features  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SEED = 42
FIRMWARE_HEADER = ROOT / "firmware" / "Services" / "Inference" / "generated" / "pm_models_data.h"


def load_dataset(data_dir: Path) -> tuple[np.ndarray, np.ndarray, list[str]]:
    files = sorted(data_dir.glob("class_*.csv"))
    if not files:
        raise SystemExit(f"no class_*.csv files in {data_dir}")
    X, y, labels = [], [], []
    for path in files:
        labels.append(re.match(r"class_(.+)\.csv", path.name)[1])
        df = pd.read_csv(path)
        if not (df["sample_rate_hz"] == 800).all():
            raise ValueError(f"{path.name}: expected 800 Hz")
        count = 0
        for (_, _), w in df.groupby(["session", "window_id"], sort=False):
            w = w.sort_values("index")
            if len(w) != N or not np.array_equal(w["index"].to_numpy(), np.arange(N)):
                raise ValueError(f"{path.name}: incomplete window")
            X.append(window_features(w[["x", "y", "z"]].to_numpy()))
            y.append(len(labels) - 1)
            count += 1
        print(f"  {path.name}: {count} windows, {df.session.nunique()} session(s)")
    return np.array(X), np.array(y), labels


def make_models() -> dict:
    return {
        "random_forest": RandomForestClassifier(
            n_estimators=150, max_depth=5, min_samples_leaf=2,
            class_weight="balanced", random_state=SEED, n_jobs=-1),
        "logistic_regression": make_pipeline(
            StandardScaler(), LogisticRegression(max_iter=3000, C=1.0)),
        "mlp": make_pipeline(
            StandardScaler(), MLPClassifier(hidden_layer_sizes=(24, 12), alpha=0.1,
                                            learning_rate_init=0.001, max_iter=1800,
                                            random_state=SEED)),
    }


# ----------------------------------------------------------------------------- export
def _c_array(ctype: str, name: str, values, fmt) -> str:
    vals = [fmt(v) for v in values]
    rows = [", ".join(vals[i:i + 8]) for i in range(0, len(vals), 8)]
    return f"static const {ctype} {name}[{len(vals)}] = {{\n  " + ",\n  ".join(rows) + ",\n};\n"


def _f(v: float) -> str:
    s = f"{float(v):.9g}"
    if "e" not in s and "." not in s and "inf" not in s:
        s += ".0"
    return s + "f"


def export_header(rf: RandomForestClassifier, lr, n_classes: int, path: Path) -> None:
    """Flatten the sklearn forest + logistic regression into C arrays."""
    scaler, logreg = lr.named_steps["standardscaler"], lr.named_steps["logisticregression"]
    # Fold standardisation into the linear model: w' = w / sigma, b' = b - sum(w * mu / sigma)
    w = logreg.coef_ / scaler.scale_
    b = logreg.intercept_ - (logreg.coef_ * scaler.mean_ / scaler.scale_).sum(axis=1)

    offsets, feat, thr, left, right, vals = [], [], [], [], [], []
    for est in rf.estimators_:
        t = est.tree_
        base = len(feat)
        offsets.append(base)
        feat += t.feature.tolist()                          # leaves: -2
        thr += t.threshold.tolist()
        left += [(c + base) if c >= 0 else -1 for c in t.children_left]
        right += [(c + base) if c >= 0 else -1 for c in t.children_right]
        v = t.value[:, 0, :]
        vals += (v / v.sum(axis=1, keepdims=True)).ravel().tolist()   # class fractions

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join([
        "#ifndef PM_MODELS_DATA_H\n#define PM_MODELS_DATA_H\n",
        f"#define PM_FEATURES {w.shape[1]}\n#define PM_CLASSES {n_classes}\n",
        _c_array("float", "pm_lr_w", w.ravel(), _f),
        _c_array("float", "pm_lr_b", b, _f),
        f"#define PM_RF_TREES {len(offsets)}\n#define PM_RF_NODES {len(feat)}\n",
        _c_array("int", "pm_rf_offsets", offsets, str),
        _c_array("int", "pm_rf_feature", feat, str),
        _c_array("float", "pm_rf_threshold", thr, _f),
        _c_array("int", "pm_rf_left", left, str),
        _c_array("int", "pm_rf_right", right, str),
        _c_array("float", "pm_rf_values", vals, _f),
        "#endif\n",
    ]))


# ----------------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", type=Path, default=ROOT / "data" / "raw")
    ap.add_argument("--out", type=Path, default=ROOT / "training" / "output")
    ap.add_argument("--deploy", action="store_true", help=f"also write {FIRMWARE_HEADER.relative_to(ROOT)}")
    args = ap.parse_args()

    print("Loading windows + extracting features ...")
    X, y, labels = load_dataset(args.data)
    print(f"Dataset: {X.shape[0]} windows x {X.shape[1]} features, {len(labels)} classes")

    X_tr, X_te, y_tr, y_te = train_test_split(X, y, test_size=0.25, stratify=y, random_state=SEED)
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=SEED)

    report: dict = {"labels": labels, "n_windows": int(len(y)), "models": {}}
    fitted = {}
    for name, model in make_models().items():
        cv_acc = cross_val_score(model, X, y, cv=cv).mean()
        model.fit(X_tr, y_tr)
        pred = model.predict(X_te)
        fitted[name] = model
        report["models"][name] = {
            "holdout_accuracy": round(accuracy_score(y_te, pred), 4),
            "holdout_balanced_accuracy": round(balanced_accuracy_score(y_te, pred), 4),
            "holdout_macro_f1": round(f1_score(y_te, pred, average="macro"), 4),
            "cv5_accuracy": round(float(cv_acc), 4),
            "holdout_confusion": confusion_matrix(y_te, pred, labels=range(len(labels))).tolist(),
        }
        print(f"\n== {name}: holdout acc {accuracy_score(y_te, pred):.3f} | 5-fold CV acc {cv_acc:.3f}")
        print(classification_report(y_te, pred, target_names=labels, digits=3, zero_division=0))

    rf = fitted["random_forest"]
    top = np.argsort(rf.feature_importances_)[::-1][:10]
    report["rf_top_features"] = [[FEATURE_NAMES[i], round(float(rf.feature_importances_[i]), 4)] for i in top]
    report["rf_nodes"] = int(sum(e.tree_.node_count for e in rf.estimators_))

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "metrics.json").write_text(json.dumps(report, indent=2))
    header = args.out / "pm_models_data.h"
    export_header(rf, fitted["logistic_regression"], len(labels), header)
    print(f"RF nodes: {report['rf_nodes']} -> {header}")
    print("\nWARNING: one session per class - scores are exploratory, not field performance.")

    if args.deploy:
        FIRMWARE_HEADER.write_text(header.read_text())
        print(f"deployed -> {FIRMWARE_HEADER}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
