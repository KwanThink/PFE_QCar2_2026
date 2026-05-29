from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.acados_interface.ocp_builder import build_ocp_solver
from src.acados_interface.sim_builder import build_sim_solver
from src.model_traj.config import load_config


# Build both Acados solvers from the YAML configuration.
def main() -> None:
    cfg = load_config()
    build_ocp_solver(cfg)
    build_sim_solver(cfg)
    print("Built Acados OCP Solver and Acados Sim Solver.")


if __name__ == "__main__":
    main()
