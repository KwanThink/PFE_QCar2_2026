from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from src.model_traj.config import load_config
from src.acados_interface.ocp_builder import build_ocp_solver
from src.acados_interface.sim_builder import build_sim_solver


# Run this module as a script with the default parameters.
# Useful for regenerating reference files from the command line.
def main() -> None:
    cfg = load_config()
    build_ocp_solver(cfg)
    build_sim_solver(cfg)
    print("Built Acados OCP Solver and Acados Sim Solver.")


if __name__ == "__main__":
    main()
