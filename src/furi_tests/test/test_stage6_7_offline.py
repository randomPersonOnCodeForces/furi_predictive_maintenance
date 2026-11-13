"""Offline tests for the Stage 6/7 predictive-maintenance utilities."""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]


def _load_module(name: str, relative_path: str):
    path = ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


generator = _load_module("stage6_generator", "tools/generate_stage6_dataset.py")
trainer = _load_module("stage7_trainer", "tools/train_stage7_isolation_forest.py")


def test_stage6_dataset_is_reproducible(tmp_path):
    first = generator.generate_dataset(200, 7, 40, 100.0)
    second = generator.generate_dataset(200, 7, 40, 100.0)
    assert first == second

    output = tmp_path / "dataset.csv"
    generator.write_csv(first, output)
    with output.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))

    assert len(rows) == 200
    assert {row["mode"] for row in rows} == set(generator.MODES)
    assert {int(row["label"]) for row in rows} == {0, 1}


def test_degradation_changes_expected_features():
    rng_a = np.random.default_rng(11)
    healthy = generator._simulate_window(rng_a, "healthy", 0.0, 100, 100.0)

    rng_b = np.random.default_rng(11)
    degraded = generator._simulate_window(
        rng_b, "combined", 0.8, 100, 100.0
    )

    assert degraded["j1_command_loss_rms"] > healthy["j1_command_loss_rms"]
    assert (
        degraded["j1_position_residual_std"]
        > healthy["j1_position_residual_std"]
    )
    assert (
        degraded["j1_velocity_residual_std"]
        > healthy["j1_velocity_residual_std"]
    )


def test_threshold_and_metrics():
    healthy_scores = np.array([0.5, 0.45, 0.40, 0.35, 0.30])
    threshold = trainer.choose_threshold(healthy_scores, 0.2)
    assert np.isfinite(threshold)

    y_true = np.array([0, 0, 1, 1])
    scores = np.array([0.5, 0.4, 0.1, 0.2])
    metrics = trainer.evaluate_predictions(y_true, scores, threshold=0.3)

    assert metrics["true_positive"] == 2
    assert metrics["true_negative"] == 2
    assert metrics["false_positive"] == 0
    assert metrics["false_negative"] == 0
    assert metrics["f1"] == 1.0
