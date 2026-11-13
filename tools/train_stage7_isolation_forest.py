#!/usr/bin/env python3
"""Train and evaluate the Stage 7 Isolation Forest baseline."""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import joblib
import numpy as np
from sklearn.ensemble import IsolationForest
from sklearn.metrics import (
    confusion_matrix,
    f1_score,
    precision_score,
    recall_score,
    roc_auc_score,
)
from sklearn.model_selection import train_test_split


META_COLUMNS = {"sample_id", "seed", "mode", "severity", "label"}


def load_dataset(path: Path) -> tuple[np.ndarray, np.ndarray, list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("dataset is empty")

    feature_names = [name for name in rows[0] if name not in META_COLUMNS]
    if not feature_names:
        raise ValueError("dataset contains no feature columns")

    x = np.asarray(
        [[float(row[name]) for name in feature_names] for row in rows],
        dtype=float,
    )
    y = np.asarray([int(row["label"]) for row in rows], dtype=int)

    if not np.all(np.isfinite(x)):
        raise ValueError("dataset contains non-finite feature values")
    if not set(np.unique(y)).issubset({0, 1}):
        raise ValueError("labels must be 0 for healthy and 1 for degraded")
    return x, y, feature_names, rows


def choose_threshold(scores: np.ndarray, contamination: float) -> float:
    if not 0.0 < contamination < 0.5:
        raise ValueError("contamination must be between 0 and 0.5")
    return float(np.quantile(scores, contamination))


def evaluate_predictions(
    y_true: np.ndarray,
    anomaly_scores: np.ndarray,
    threshold: float,
) -> dict[str, float | int]:
    y_pred = (anomaly_scores < threshold).astype(int)
    tn, fp, fn, tp = confusion_matrix(y_true, y_pred, labels=[0, 1]).ravel()

    metrics: dict[str, float | int] = {
        "true_negative": int(tn),
        "false_positive": int(fp),
        "false_negative": int(fn),
        "true_positive": int(tp),
        "precision": float(precision_score(y_true, y_pred, zero_division=0)),
        "recall": float(recall_score(y_true, y_pred, zero_division=0)),
        "f1": float(f1_score(y_true, y_pred, zero_division=0)),
    }
    if len(np.unique(y_true)) == 2:
        metrics["roc_auc"] = float(roc_auc_score(y_true, -anomaly_scores))
    return metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("data/stage6_predictive_maintenance.csv"),
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("artifacts/stage7_isolation_forest.joblib"),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("artifacts/stage7_metrics.json"),
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--contamination", type=float, default=0.03)
    parser.add_argument("--test-size", type=float, default=0.30)
    parser.add_argument("--estimators", type=int, default=200)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    x, y, feature_names, rows = load_dataset(args.input)

    healthy_indices = np.flatnonzero(y == 0)
    degraded_indices = np.flatnonzero(y == 1)
    if healthy_indices.size < 10 or degraded_indices.size < 10:
        raise ValueError("dataset must contain healthy and degraded examples")

    healthy_train, healthy_test = train_test_split(
        healthy_indices,
        test_size=args.test_size,
        random_state=args.seed,
        shuffle=True,
    )
    _, degraded_test = train_test_split(
        degraded_indices,
        test_size=args.test_size,
        random_state=args.seed,
        shuffle=True,
    )

    model = IsolationForest(
        n_estimators=args.estimators,
        contamination="auto",
        random_state=args.seed,
        n_jobs=-1,
    )
    model.fit(x[healthy_train])

    healthy_train_scores = model.score_samples(x[healthy_train])
    threshold = choose_threshold(healthy_train_scores, args.contamination)

    test_indices = np.concatenate([healthy_test, degraded_test])
    rng = np.random.default_rng(args.seed)
    rng.shuffle(test_indices)

    start = time.perf_counter()
    scores = model.score_samples(x[test_indices])
    elapsed = time.perf_counter() - start
    latency_ms = elapsed * 1000.0 / max(1, len(test_indices))

    metrics = evaluate_predictions(y[test_indices], scores, threshold)
    metrics.update(
        {
            "dataset_rows": int(len(rows)),
            "feature_count": int(len(feature_names)),
            "healthy_train_rows": int(len(healthy_train)),
            "healthy_test_rows": int(len(healthy_test)),
            "degraded_test_rows": int(len(degraded_test)),
            "threshold": threshold,
            "contamination_quantile": float(args.contamination),
            "mean_inference_latency_ms_per_sample": float(latency_ms),
            "seed": int(args.seed),
        }
    )

    bundle = {
        "model": model,
        "feature_names": feature_names,
        "threshold": threshold,
        "metadata": {
            "training_source": "synthetic_stage6",
            "seed": int(args.seed),
        },
    }

    args.model.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(bundle, args.model)
    args.report.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(json.dumps(metrics, indent=2, sort_keys=True))
    print(f"Saved model to {args.model}")
    print(f"Saved report to {args.report}")


if __name__ == "__main__":
    main()
