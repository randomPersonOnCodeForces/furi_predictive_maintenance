# Stage 6/7 — Predictive Maintenance and Machine Learning

This branch extends the Stage 4 software pipeline into a reproducible predictive-maintenance workflow.

## Scope

Stage 6:
- generate labeled synthetic operating data from the existing four-joint command/fault model;
- model progressive command attenuation, sensor noise, and combined degradation;
- preserve healthy and degraded operating classes;
- export a 10,000+ row feature dataset for downstream modeling.

Stage 7:
- train an Isolation Forest anomaly detector on healthy operating windows;
- evaluate healthy-vs-degraded detection performance;
- report precision, recall, F1 score, confusion counts, ROC AUC when available, and per-sample inference latency;
- save the fitted model and evaluation summary.

## Data status

The current dataset is generated from the software model and is used as the working Stage 6/7 research dataset while hardware integration is unavailable. It is not a record of physical Dobot or BNO055 measurements. Hardware validation can later replace or recalibrate the simulated parameter ranges without changing the ML workflow.

## Generate the Stage 6 dataset

From the repository root:

```bash
python3 tools/generate_stage6_dataset.py \
  --output data/stage6_predictive_maintenance.csv \
  --rows 12000 \
  --seed 42
```

The script generates four operating modes:
- `healthy`
- `command_attenuation`
- `sensor_noise`
- `combined`

Each row represents one simulated operating window and contains per-joint command, tracking, position, velocity, and sensor-residual features plus the degradation severity and class label.

## Train and evaluate Stage 7

Install the Python ML dependencies if needed:

```bash
python3 -m pip install scikit-learn joblib
```

Then run:

```bash
python3 tools/train_stage7_isolation_forest.py \
  --input data/stage6_predictive_maintenance.csv \
  --model artifacts/stage7_isolation_forest.joblib \
  --report artifacts/stage7_metrics.json \
  --seed 42
```

The model is fit only on healthy training rows. The anomaly threshold is selected from the healthy training-score distribution, then evaluated against held-out healthy and degraded rows.

## Reproducibility

Both tools accept a random seed. Generated datasets include the seed, operating mode, severity, and binary label so experiments can be repeated and audited.

## Hardware follow-up

Once the physical Dobot and BNO055 path is online:
1. collect hardware command and joint-state/IMU data using the same operating-window convention;
2. compare feature distributions against the synthetic dataset;
3. adjust degradation/noise ranges where simulation is not representative;
4. retrain and report synthetic-only, hardware-only, and mixed-data results separately.
