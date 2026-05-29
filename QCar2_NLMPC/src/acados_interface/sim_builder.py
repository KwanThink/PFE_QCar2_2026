from __future__ import annotations

from model_traj.config import path_from_config
from model_traj.qcar2_nlmpc_controller import export_qcar2_model


# Configure the acados simulation/integration problem.
# The same QCar2 model is used for one-step propagation.
def build_sim(cfg: dict):
    from acados_template import AcadosSim

    sim = AcadosSim()
    sim.model = export_qcar2_model(L=float(cfg["qcar2"]["L"]))
    sim.solver_options.T = float(cfg["qcar2"]["Ts"])

    acados_cfg = cfg.get("acados", {})
    sim.solver_options.integrator_type = acados_cfg.get("integrator_type", "ERK")
    sim.solver_options.num_stages = int(acados_cfg.get("sim_method_num_stages", 4))
    sim.solver_options.num_steps = int(acados_cfg.get("sim_method_num_steps", 1))

    code_export_dir = path_from_config(cfg, "generated_sim_dir")
    code_export_dir.mkdir(parents=True, exist_ok=True)
    sim.code_export_directory = str(code_export_dir)
    return sim


# Create an acados simulator from the prepared sim problem.
# Generation and compilation can be enabled or skipped as needed.
def create_sim_solver(cfg: dict, generate: bool = True, build: bool = True):
    from acados_template import AcadosSimSolver

    sim = build_sim(cfg)
    json_file = path_from_config(cfg, "generated_sim_dir") / cfg.get("acados", {}).get(
        "sim_json_file", "acados_sim_qcar2.json"
    )
    return AcadosSimSolver(sim, json_file=str(json_file), generate=generate, build=build)


# Build and return the default acados simulator.
# This wrapper is used by closed-loop simulations.
def build_sim_solver(cfg: dict):
    return create_sim_solver(cfg, generate=True, build=True)
