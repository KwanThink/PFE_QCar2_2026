from __future__ import annotations

import json
from pathlib import Path

import numpy as np


# Compute basic tracking and solver-status metrics.
# The output is saved with each closed-loop run.
def compute_tracking_metrics(states: np.ndarray, reference_states: np.ndarray, controls: np.ndarray, statuses: np.ndarray) -> dict:
    states = np.asarray(states, dtype=float)
    reference_states = np.asarray(reference_states, dtype=float)
    controls = np.asarray(controls, dtype=float)
    n = min(len(states), len(reference_states))
    err = states[:n] - reference_states[:n]

    pos_err = np.linalg.norm(err[:, :2], axis=1) if n else np.array([])
    metrics = {
        "num_steps": int(max(len(controls), 0)),
        "num_solver_failures": int(np.count_nonzero(np.asarray(statuses) != 0)),
        "rmse_x": float(np.sqrt(np.mean(err[:, 0] ** 2))) if n else float("nan"),
        "rmse_y": float(np.sqrt(np.mean(err[:, 1] ** 2))) if n else float("nan"),
        "rmse_position": float(np.sqrt(np.mean(pos_err ** 2))) if n else float("nan"),
        "max_position_error": float(np.max(pos_err)) if n else float("nan"),
        "max_abs_v": float(np.max(np.abs(controls[:, 0]))) if len(controls) else 0.0,
        "max_abs_omega_s": float(np.max(np.abs(controls[:, 1]))) if len(controls) else 0.0,
    }
    return metrics


# Save tracking metrics as a JSON file.
# Parent folders are created automatically.
def save_metrics(metrics: dict, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
