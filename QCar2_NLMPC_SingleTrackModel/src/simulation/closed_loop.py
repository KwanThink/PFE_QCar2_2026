from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from src.acados_interface.sim_builder import create_sim_solver
from src.model_traj.config import path_from_config
from src.model_traj.qcar2_single_track_nlmpc_controller import QCar2SingleTrackNLMPC
from src.model_traj.reference_loader import ReferenceTrajectory
from src.simulation.metrics import compute_tracking_metrics, save_metrics
from src.simulation.plots import save_all_plots


@dataclass
class ClosedLoopResult:
    run_dir: Path
    time_states: np.ndarray
    states: np.ndarray
    reference_states: np.ndarray
    time_controls: np.ndarray
    controls: np.ndarray
    reference_controls: np.ndarray
    solve_times: np.ndarray
    statuses: np.ndarray
    metrics: dict


# Create a timestamped folder for one simulation run.
def _make_run_dir(cfg: dict) -> Path:
    results_dir = path_from_config(cfg, "results_dir") / "runs"
    prefix = cfg.get("simulation", {}).get("run_name_prefix", "run")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"{prefix}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


# Read the initial state from YAML or from the reference.
def _initial_state(cfg: dict, reference: ReferenceTrajectory) -> np.ndarray:
    initial = cfg.get("simulation", {}).get("initial_state", "reference")
    if initial == "reference":
        return reference.q_ref[0].copy()
    return np.asarray(initial, dtype=float).reshape(4)


# Read the initial input guess from YAML or from the reference.
def _initial_input(cfg: dict, reference: ReferenceTrajectory) -> np.ndarray:
    initial = cfg.get("simulation", {}).get("initial_input", "reference")
    if initial == "reference":
        return reference.u_ref[0].copy()
    return np.asarray(initial, dtype=float).reshape(2)


# Simulate the model for one sampling step with input u.
def _simulate_one_step(sim_solver, x: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, int]:
    sim_solver.set("x", np.asarray(x, dtype=float))
    sim_solver.set("u", np.asarray(u, dtype=float))
    status = int(sim_solver.solve())
    if status == 0:
        return np.asarray(sim_solver.get("x"), dtype=float).reshape(-1), status
    return np.asarray(x, dtype=float).reshape(-1), status


# Run the MPC loop, simulate the plant, and save results.
def run_closed_loop(cfg: dict, reference: ReferenceTrajectory, plot: bool = True) -> ClosedLoopResult:
    N = int(cfg["mpc"]["N"])
    Ts = float(cfg["qcar2_single_track"]["Ts"])
    max_available_steps = max(reference.n_samples - 1, 0)
    max_steps_cfg = cfg.get("simulation", {}).get("max_steps", None)
    n_steps = max_available_steps if max_steps_cfg is None else min(int(max_steps_cfg), max_available_steps)
    n_steps = max(n_steps, 0)

    controller = QCar2SingleTrackNLMPC(cfg)
    sim_solver = create_sim_solver(cfg, generate=True, build=True)
    stop_on_failure = bool(cfg.get("simulation", {}).get("stop_on_solver_failure", True))

    x = _initial_state(cfg, reference)
    u_initial_guess = _initial_input(cfg, reference)

    states = [x.copy()]
    reference_states = [reference.q_ref[0].copy()]
    controls = []
    reference_controls = []
    solve_times = []
    statuses = []

    for k in range(n_steps):
        _, q_h, u_h = reference.horizon(k, N)
        first_input_guess = u_initial_guess if k == 0 else None
        u0, info = controller.solve(x, q_h, u_h, u_initial_guess=first_input_guess)
        statuses.append(info.status)
        solve_times.append(info.solve_time)

        if info.status != 0 and stop_on_failure:
            break

        controls.append(u0.copy())
        reference_controls.append(reference.u_ref[min(k, reference.n_samples - 1)].copy())
        x, sim_status = _simulate_one_step(sim_solver, x, u0)
        if sim_status != 0:
            statuses[-1] = sim_status
            if stop_on_failure:
                break
        states.append(x.copy())
        reference_states.append(reference.q_ref[min(k + 1, reference.n_samples - 1)].copy())

    time_states = np.arange(len(states), dtype=float) * Ts
    time_controls = np.arange(len(controls), dtype=float) * Ts
    states_arr = np.asarray(states, dtype=float)
    ref_states_arr = np.asarray(reference_states, dtype=float)
    controls_arr = np.asarray(controls, dtype=float).reshape(-1, 2)
    ref_controls_arr = np.asarray(reference_controls, dtype=float).reshape(-1, 2)
    solve_times_arr = np.asarray(solve_times, dtype=float)
    statuses_arr = np.asarray(statuses, dtype=int)

    metrics = compute_tracking_metrics(states_arr, ref_states_arr, controls_arr, statuses_arr)
    run_dir = _make_run_dir(cfg)

    np.savetxt(
        run_dir / "states.csv",
        np.column_stack((time_states, states_arr)),
        delimiter=",",
        header="t,X,Y,psi,vx",
        comments="",
    )
    np.savetxt(
        run_dir / "controls.csv",
        np.column_stack((time_controls, controls_arr)),
        delimiter=",",
        header="t,delta,ax",
        comments="",
    )
    np.savetxt(
        run_dir / "reference_used.csv",
        np.column_stack((time_states, ref_states_arr)),
        delimiter=",",
        header="t,X_ref,Y_ref,psi_ref,vx_ref",
        comments="",
    )
    np.savetxt(
        run_dir / "reference_controls.csv",
        np.column_stack((time_controls, ref_controls_arr)),
        delimiter=",",
        header="t,delta_ref,ax_ref",
        comments="",
    )
    np.savetxt(
        run_dir / "solve_times.csv",
        np.column_stack((np.arange(len(solve_times_arr), dtype=float) * Ts, solve_times_arr, statuses_arr)),
        delimiter=",",
        header="t,solve_time,status",
        comments="",
    )
    save_metrics(metrics, run_dir / "metrics.json")

    if plot and len(controls_arr):
        save_all_plots(
            run_dir,
            time_states,
            states_arr,
            ref_states_arr,
            time_controls,
            controls_arr,
            ref_controls_arr,
            solve_times_arr,
        )

    return ClosedLoopResult(
        run_dir=run_dir,
        time_states=time_states,
        states=states_arr,
        reference_states=ref_states_arr,
        time_controls=time_controls,
        controls=controls_arr,
        reference_controls=ref_controls_arr,
        solve_times=solve_times_arr,
        statuses=statuses_arr,
        metrics=metrics,
    )
