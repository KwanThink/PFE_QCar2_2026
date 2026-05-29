from __future__ import annotations

import numpy as np

from src.model_traj.config import path_from_config
from src.model_traj.qcar2_single_track_nlmpc_controller import export_qcar2_single_track_model


# Create a diagonal matrix from a list of weights.
def _diag(values: list[float] | tuple[float, ...]) -> np.ndarray:
    return np.diag(np.asarray(values, dtype=float))


# Build the Acados OCP definition for the NLMPC problem.
def build_ocp(cfg: dict):
    from acados_template import AcadosOcp

    Lwb = float(cfg["qcar2_single_track"]["Lwb"])
    N = int(cfg["mpc"]["N"])
    tf = float(cfg["mpc"].get("tf", N * float(cfg["qcar2_single_track"]["Ts"])))

    model = export_qcar2_single_track_model(Lwb=Lwb)
    ocp = AcadosOcp()
    ocp.model = model

    nx = int(model.x.size()[0])
    nu = int(model.u.size()[0])
    ny = nx + nu

    Q = _diag(cfg["mpc"]["Q"])
    R = _diag(cfg["mpc"]["R"])
    Qe = _diag(cfg["mpc"]["Qe"])

    ocp.dims.N = N
    ocp.solver_options.tf = tf

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = np.block(
        [
            [Q, np.zeros((nx, nu))],
            [np.zeros((nu, nx)), R],
        ]
    )
    ocp.cost.W_e = Qe
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    constraints = cfg["constraints"]
    ocp.constraints.lbu = np.array([constraints["delta_min"], constraints["ax_min"]], dtype=float)
    ocp.constraints.ubu = np.array([constraints["delta_max"], constraints["ax_max"]], dtype=float)
    ocp.constraints.idxbu = np.array([0, 1], dtype=int)

    # Limit the longitudinal velocity state vx.
    ocp.constraints.idxbx = np.array([3], dtype=int)
    ocp.constraints.lbx = np.array([constraints["vx_min"]], dtype=float)
    ocp.constraints.ubx = np.array([constraints["vx_max"]], dtype=float)
    ocp.constraints.x0 = np.zeros(nx)

    acados_cfg = cfg.get("acados", {})
    ocp.solver_options.qp_solver = acados_cfg.get("qp_solver", "PARTIAL_CONDENSING_HPIPM")
    ocp.solver_options.hessian_approx = acados_cfg.get("hessian_approx", "GAUSS_NEWTON")
    ocp.solver_options.integrator_type = acados_cfg.get("integrator_type", "ERK")
    ocp.solver_options.nlp_solver_type = acados_cfg.get("nlp_solver_type", "SQP_RTI")
    ocp.solver_options.sim_method_num_stages = int(acados_cfg.get("sim_method_num_stages", 4))
    ocp.solver_options.sim_method_num_steps = int(acados_cfg.get("sim_method_num_steps", 1))
    ocp.solver_options.print_level = int(acados_cfg.get("print_level", 0))

    code_export_dir = path_from_config(cfg, "generated_ocp_dir")
    code_export_dir.mkdir(parents=True, exist_ok=True)
    ocp.code_export_directory = str(code_export_dir)
    return ocp


# Create an Acados OCP solver from the YAML configuration.
def create_ocp_solver(cfg: dict, generate: bool = True, build: bool = True):
    from acados_template import AcadosOcpSolver

    ocp = build_ocp(cfg)
    json_file = path_from_config(cfg, "generated_ocp_dir") / cfg.get("acados", {}).get(
        "ocp_json_file", "acados_ocp_qcar2_single_track.json"
    )
    return AcadosOcpSolver(ocp, json_file=str(json_file), generate=generate, build=build)


# Generate and build the Acados OCP solver.
def build_ocp_solver(cfg: dict):
    return create_ocp_solver(cfg, generate=True, build=True)
