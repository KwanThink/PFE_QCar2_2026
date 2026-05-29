from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class ReferenceTrajectory:
    t: np.ndarray
    q_ref: np.ndarray
    u_ref: np.ndarray
    raw: np.ndarray
    state_columns: tuple[str, ...]
    input_columns: tuple[str, ...]

    @property
    # Return the number of reference samples available.
    # This is inferred from the state reference array.
    def n_samples(self) -> int:
        return int(self.q_ref.shape[0])

    @property
    # Return the state dimension of the reference trajectory.
    # It is used to size MPC reference arrays.
    def nx(self) -> int:
        return int(self.q_ref.shape[1])

    @property
    # Return the input dimension of the reference trajectory.
    # It is used to size control reference arrays.
    def nu(self) -> int:
        return int(self.u_ref.shape[1])

    # Find the nearest reference index for a requested time.
    # The result is clipped to stay inside the trajectory.
    def index_at_time(self, time_value: float) -> int:
        return int(np.clip(np.searchsorted(self.t, time_value, side="left"), 0, self.n_samples - 1))

    # Return one state and input reference sample.
    # This is used for pointwise tracking and initialization.
    def sample(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        idx = int(np.clip(index, 0, self.n_samples - 1))
        return self.q_ref[idx].copy(), self.u_ref[idx].copy()

    # Build a finite MPC reference horizon from a start index.
    # The last reference is repeated if the horizon reaches the end.
    def horizon(self, start_index: int, N: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        idx_q = np.clip(np.arange(start_index, start_index + N + 1), 0, self.n_samples - 1)
        idx_u = np.clip(np.arange(start_index, start_index + N), 0, self.n_samples - 1)
        return self.t[idx_q], self.q_ref[idx_q], self.u_ref[idx_u]


# Load a generated reference CSV into structured arrays.
# Columns are mapped to state and input references for MPC.
def load_reference_csv(
    csv_path: str | Path,
    state_columns: Iterable[str] = ("x", "y", "theta", "varphi"),
    input_columns: Iterable[str] = ("v", "omega_s"),
) -> ReferenceTrajectory:
    csv_path = Path(csv_path)
    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=float)
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)

    state_columns = tuple(state_columns)
    input_columns = tuple(input_columns)

    t = np.asarray(data["t"], dtype=float)
    q_ref = np.column_stack([np.asarray(data[col], dtype=float) for col in state_columns])
    u_ref = np.column_stack([np.asarray(data[col], dtype=float) for col in input_columns])
    return ReferenceTrajectory(t=t, q_ref=q_ref, u_ref=u_ref, raw=data, state_columns=state_columns, input_columns=input_columns)


# Load the reference trajectory specified in the config file.
# This is the standard entry point for simulations.
def load_reference_from_config(cfg: dict) -> ReferenceTrajectory:
    from .config import path_from_config

    reference_dir = path_from_config(cfg, "reference_dir")
    csv_name = cfg["trajectory"]["csv_file"]
    state_columns = cfg["trajectory"]["columns"]["state"]
    input_columns = cfg["trajectory"]["columns"]["input"]
    return load_reference_csv(reference_dir / csv_name, state_columns, input_columns)
