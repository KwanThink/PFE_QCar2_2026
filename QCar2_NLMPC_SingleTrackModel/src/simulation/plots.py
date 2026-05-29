from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# Plot the XY trajectory against the reference.
def plot_tracking(states: np.ndarray, reference_states: np.ndarray, output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure()
    plt.plot(reference_states[:, 0], reference_states[:, 1], "--", label="reference")
    plt.plot(states[:, 0], states[:, 1], label="NLMPC + Acados Sim")
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("QCar2 single-track trajectory tracking")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()


# Plot all state variables over time.
def plot_states(time: np.ndarray, states: np.ndarray, reference_states: np.ndarray, output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    labels = ["X [m]", "Y [m]", "psi [rad]", "vx [m/s]"]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
    for i, ax in enumerate(axes):
        ax.plot(time, reference_states[:, i], "--", label="reference")
        ax.plot(time, states[:, i], label="state")
        ax.set_ylabel(labels[i])
        ax.grid(True)
    axes[-1].set_xlabel("t [s]")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


# Plot control inputs over time.
def plot_controls(time: np.ndarray, controls: np.ndarray, reference_controls: np.ndarray, output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    labels = ["delta [rad]", "ax [m/s^2]"]
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 5))
    for i, ax in enumerate(axes):
        ax.plot(time, reference_controls[:, i], "--", label="reference")
        ax.plot(time, controls[:, i], label="control")
        ax.set_ylabel(labels[i])
        ax.grid(True)
    axes[-1].set_xlabel("t [s]")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


# Plot the MPC solve time over time.
def plot_solve_times(time: np.ndarray, solve_times: np.ndarray, output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure()
    plt.plot(time, 1e3 * np.asarray(solve_times, dtype=float))
    plt.grid(True)
    plt.xlabel("t [s]")
    plt.ylabel("solve time [ms]")
    plt.title("Acados OCP solve time")
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()


# Save all plots for one closed-loop run.
def save_all_plots(
    run_dir: str | Path,
    time_states: np.ndarray,
    states: np.ndarray,
    reference_states: np.ndarray,
    time_controls: np.ndarray,
    controls: np.ndarray,
    reference_controls: np.ndarray,
    solve_times: np.ndarray,
) -> None:
    run_dir = Path(run_dir)
    plot_tracking(states, reference_states, run_dir / "trajectory_tracking.png")
    plot_states(time_states, states, reference_states, run_dir / "states.png")
    plot_controls(time_controls, controls, reference_controls, run_dir / "controls.png")
    plot_solve_times(time_controls, solve_times, run_dir / "solve_times.png")
