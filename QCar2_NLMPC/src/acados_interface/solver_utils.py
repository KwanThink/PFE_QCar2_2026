from __future__ import annotations

import numpy as np


# Fix the initial state constraint of the OCP solver.
# The current measured or simulated state is imposed at stage 0.
def set_initial_state(ocp_solver, x0: np.ndarray) -> None:
    x0 = np.asarray(x0, dtype=float).reshape(-1)
    ocp_solver.set(0, "lbx", x0)
    ocp_solver.set(0, "ubx", x0)


# Set state and input references over the MPC horizon.
# The terminal reference uses only the state component.
def set_reference_horizon(ocp_solver, q_ref: np.ndarray, u_ref: np.ndarray) -> None:
    q_ref = np.asarray(q_ref, dtype=float)
    u_ref = np.asarray(u_ref, dtype=float)
    N = u_ref.shape[0]
    for i in range(N):
        ocp_solver.set(i, "yref", np.concatenate((q_ref[i], u_ref[i])))
    ocp_solver.set(N, "yref", q_ref[N])


# Warm-start the solver with reference states and inputs.
# This gives the optimizer a feasible-looking initial guess.
def initialize_state_guess(ocp_solver, q_ref: np.ndarray, u_ref: np.ndarray) -> None:
    q_ref = np.asarray(q_ref, dtype=float)
    u_ref = np.asarray(u_ref, dtype=float)
    N = u_ref.shape[0]
    for i in range(N):
        ocp_solver.set(i, "x", q_ref[i])
        ocp_solver.set(i, "u", u_ref[i])
    ocp_solver.set(N, "x", q_ref[N])


# Read the latest acados solver runtime statistic.
# Returns NaN if the statistic is unavailable.
def get_solver_time(ocp_solver) -> float:
    return float(ocp_solver.get_stats("time_tot"))
