from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from src.model_traj.config import load_config
from src.model_traj.reference_loader import load_reference_from_config
from src.simulation.closed_loop import run_closed_loop


# Run this module as a script with the default parameters.
# Useful for regenerating reference files from the command line.
def main() -> None:
    cfg = load_config()
    reference = load_reference_from_config(cfg)
    result = run_closed_loop(cfg, reference, plot=True)
    print(f"Closed-loop run saved to: {result.run_dir}")
    print(f"RMSE position: {result.metrics['rmse_position']:.6f} m")
    print(f"Max position error: {result.metrics['max_position_error']:.6f} m")
    print(f"Solver failures: {result.metrics['num_solver_failures']}")


if __name__ == "__main__":
    main()
