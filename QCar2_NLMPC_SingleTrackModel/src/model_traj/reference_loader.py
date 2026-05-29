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

    # Return the number of samples in the reference trajectory.
    @property
    def n_samples(self) -> int:
        return int(self.q_ref.shape[0])

    # Return the number of state variables.
    @property
    def nx(self) -> int:
        return int(self.q_ref.shape[1])

    # Return the number of input variables.
    @property
    def nu(self) -> int:
        return int(self.u_ref.shape[1])

    # Find the closest reference index for a given time.
    def index_at_time(self, time_value: float) -> int:
        return int(np.clip(np.searchsorted(self.t, time_value, side="left"), 0, self.n_samples - 1))

    # Return the state and input reference at one index.
    def sample(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        idx = int(np.clip(index, 0, self.n_samples - 1))
        return self.q_ref[idx].copy(), self.u_ref[idx].copy()

    # Return the reference section used by the MPC horizon.
    def horizon(self, start_index: int, N: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        idx_q = np.clip(np.arange(start_index, start_index + N + 1), 0, self.n_samples - 1)
        idx_u = np.clip(np.arange(start_index, start_index + N), 0, self.n_samples - 1)
        return self.t[idx_q], self.q_ref[idx_q], self.u_ref[idx_u]


# Load a CSV reference and split it into state and input arrays.
def load_reference_csv(
    csv_path: str | Path,
    state_columns: Iterable[str] = ("X", "Y", "psi", "vx"),
    input_columns: Iterable[str] = ("delta", "ax"),
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


# Load the reference trajectory declared in the YAML configuration.
def load_reference_from_config(cfg: dict) -> ReferenceTrajectory:
    from src.model_traj.config import path_from_config

    reference_dir = path_from_config(cfg, "reference_dir")
    csv_name = cfg["trajectory"]["csv_file"]
    state_columns = cfg["trajectory"]["columns"]["state"]
    input_columns = cfg["trajectory"]["columns"]["input"]
    return load_reference_csv(reference_dir / csv_name, state_columns, input_columns)
