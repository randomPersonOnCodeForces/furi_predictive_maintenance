#!/usr/bin/env python3
"""Generate a reproducible Stage 6 predictive-maintenance feature dataset."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


JOINT_COUNT = 4
VELOCITY_LIMITS = np.array([1.0, 1.0, 1.0, 1.5], dtype=float)
BASE_AMPLITUDES = np.array([0.20, 0.15, 0.10, 0.10], dtype=float)
BASE_FREQUENCIES = np.array([0.10, 0.15, 0.20, 0.25], dtype=float)
BASE_POSITION_NOISE = np.array([0.002] * JOINT_COUNT, dtype=float)
BASE_VELOCITY_NOISE = np.array([0.005] * JOINT_COUNT, dtype=float)

MODES = ("healthy", "command_attenuation", "sensor_noise", "combined")


def _rms(values: np.ndarray, axis: int = 0) -> np.ndarray:
    return np.sqrt(np.mean(np.square(values), axis=axis))


def _simulate_window(
    rng: np.random.Generator,
    mode: str,
    severity: float,
    samples: int,
    rate_hz: float,
) -> dict[str, float | int | str]:
    dt = 1.0 / rate_hz
    t = np.arange(samples, dtype=float) * dt

    phase = rng.uniform(0.0, 2.0 * math.pi, size=JOINT_COUNT)
    amplitude_scale = rng.uniform(0.85, 1.15, size=JOINT_COUNT)
    frequency_scale = rng.uniform(0.90, 1.10, size=JOINT_COUNT)

    amplitude = BASE_AMPLITUDES * amplitude_scale
    frequency = BASE_FREQUENCIES * frequency_scale

    raw = np.column_stack(
        [
            amplitude[j] * np.sin(2.0 * math.pi * frequency[j] * t + phase[j])
            for j in range(JOINT_COUNT)
        ]
    )
    raw = np.clip(raw, -VELOCITY_LIMITS, VELOCITY_LIMITS)

    applied = raw.copy()
    attenuation_active = mode in ("command_attenuation", "combined")
    noise_active = mode in ("sensor_noise", "combined")

    if attenuation_active:
        viscous = severity * np.array([0.10, 0.14, 0.18, 0.12], dtype=float)
        coulomb = severity * np.array([0.012, 0.010, 0.008, 0.010], dtype=float)
        smoothing = 0.02
        opposing = viscous * raw + coulomb * np.tanh(raw / smoothing)
        applied = np.sign(raw) * np.maximum(0.0, np.abs(raw) - np.abs(opposing))

    clean_velocity = applied.copy()
    clean_position = np.cumsum(clean_velocity, axis=0) * dt
    clean_position -= np.mean(clean_position, axis=0, keepdims=True)

    position_std = BASE_POSITION_NOISE.copy()
    velocity_std = BASE_VELOCITY_NOISE.copy()

    if noise_active:
        position_std *= 1.0 + 12.0 * severity
        velocity_std *= 1.0 + 10.0 * severity

    observed_position = clean_position + rng.normal(
        0.0, position_std, size=clean_position.shape
    )
    observed_velocity = clean_velocity + rng.normal(
        0.0, velocity_std, size=clean_velocity.shape
    )

    record: dict[str, float | int | str] = {
        "mode": mode,
        "severity": float(severity),
        "label": 0 if mode == "healthy" else 1,
    }

    for j in range(JOINT_COUNT):
        idx = j + 1
        command_loss = raw[:, j] - applied[:, j]
        position_residual = observed_position[:, j] - clean_position[:, j]
        velocity_residual = observed_velocity[:, j] - clean_velocity[:, j]

        record[f"j{idx}_raw_command_rms"] = float(_rms(raw[:, j]))
        record[f"j{idx}_applied_command_rms"] = float(_rms(applied[:, j]))
        record[f"j{idx}_command_loss_rms"] = float(_rms(command_loss))
        record[f"j{idx}_position_std"] = float(np.std(observed_position[:, j]))
        record[f"j{idx}_velocity_rms"] = float(_rms(observed_velocity[:, j]))
        record[f"j{idx}_position_residual_std"] = float(np.std(position_residual))
        record[f"j{idx}_velocity_residual_std"] = float(np.std(velocity_residual))
        record[f"j{idx}_tracking_rmse"] = float(
            _rms(observed_velocity[:, j] - raw[:, j])
        )

    return record


def generate_dataset(
    rows: int,
    seed: int,
    samples_per_window: int,
    rate_hz: float,
) -> list[dict[str, float | int | str]]:
    if rows < 100:
        raise ValueError("rows must be at least 100")
    if samples_per_window < 10:
        raise ValueError("samples_per_window must be at least 10")
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")

    rng = np.random.default_rng(seed)
    records: list[dict[str, float | int | str]] = []

    for index in range(rows):
        mode = MODES[index % len(MODES)]
        if mode == "healthy":
            severity = 0.0
        else:
            severity = float(rng.uniform(0.10, 1.0))

        record = _simulate_window(
            rng=rng,
            mode=mode,
            severity=severity,
            samples=samples_per_window,
            rate_hz=rate_hz,
        )
        record["sample_id"] = index
        record["seed"] = seed
        records.append(record)

    rng.shuffle(records)
    return records


def write_csv(records: list[dict[str, float | int | str]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(records[0].keys())
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/stage6_predictive_maintenance.csv"),
    )
    parser.add_argument("--rows", type=int, default=12000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--samples-per-window", type=int, default=100)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    records = generate_dataset(
        rows=args.rows,
        seed=args.seed,
        samples_per_window=args.samples_per_window,
        rate_hz=args.rate_hz,
    )
    write_csv(records, args.output)

    healthy = sum(int(row["label"]) == 0 for row in records)
    degraded = len(records) - healthy
    print(f"Wrote {len(records)} rows to {args.output}")
    print(f"Healthy: {healthy}; degraded: {degraded}")


if __name__ == "__main__":
    main()
