from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from acados_interface.solver_utils import (
    get_solver_time,
    initialize_state_guess,
    set_initial_state,
    set_reference_horizon,
)


# Create the symbolic QCar2 kinematic model for acados.
# States and inputs match the generated reference trajectory.
def export_qcar2_model(L: float):
    from acados_template import AcadosModel
    from casadi import SX, cos, sin, tan, vertcat

    model = AcadosModel()
    model.name = "qcar2_kinematic"

    x = SX.sym("x")
    y = SX.sym("y")
    theta = SX.sym("theta")
    phi = SX.sym("phi")
    states = vertcat(x, y, theta, phi)

    v = SX.sym("v")
    omega_s = SX.sym("omega_s")
    controls = vertcat(v, omega_s)

    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    theta_dot = SX.sym("theta_dot")
    phi_dot = SX.sym("phi_dot")
    xdot = vertcat(x_dot, y_dot, theta_dot, phi_dot)

    f_expl = vertcat(
        v * cos(theta),
        v * sin(theta),
        (v / L) * tan(phi),
        omega_s,
    )
    f_impl = xdot - f_expl

    model.x = states
    model.u = controls
    model.xdot = xdot
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    return model


@dataclass
class ControlInfo:
    status: int
    solve_time: float


class QCar2NLMPC:
    # Initialize the controller and keep references to config and solver.
    # Solver creation is skipped when an existing solver is provided.
    def __init__(self, cfg: dict, ocp_solver=None):
        self.cfg = cfg
        self.N = int(cfg["mpc"]["N"])
        self.nx = 4
        self.nu = 2
        if ocp_solver is None:
            from acados_interface.ocp_builder import create_ocp_solver

            self.solver = create_ocp_solver(cfg, generate=True, build=True)
        else:
            self.solver = ocp_solver
        self._initialized_guess = False

    # Solve one NMPC problem from the current state and references.
    # The first control input is returned for closed-loop simulation.
    def solve(self, x_current: np.ndarray, q_ref_horizon: np.ndarray, u_ref_horizon: np.ndarray, 
               u_initial_guess: np.ndarray | None = None,) -> tuple[np.ndarray, ControlInfo]:
        x_current = np.asarray(x_current, dtype=float).reshape(self.nx)
        q_ref_horizon = np.asarray(q_ref_horizon, dtype=float).reshape(self.N + 1, self.nx)
        u_ref_horizon = np.asarray(u_ref_horizon, dtype=float).reshape(self.N, self.nu)
        if u_initial_guess is not None:
            u_initial_guess = np.asarray(u_initial_guess, dtype=float).reshape(self.nu)

        set_initial_state(self.solver, x_current)
        set_reference_horizon(self.solver, q_ref_horizon, u_ref_horizon)
        if not self._initialized_guess:
            initialize_state_guess(self.solver, q_ref_horizon, u_ref_horizon)
            self.solver.set(0, "x", x_current)
            if u_initial_guess is not None:
                self.solver.set(0, "u", u_initial_guess)
            self._initialized_guess = True

        status = int(self.solver.solve())
        solve_time = get_solver_time(self.solver)
        if status != 0:
            return np.zeros(self.nu), ControlInfo(status=status, solve_time=solve_time)

        u0 = np.asarray(self.solver.get(0, "u"), dtype=float).reshape(self.nu)
        return u0, ControlInfo(status=status, solve_time=solve_time)
