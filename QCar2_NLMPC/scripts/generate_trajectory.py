from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from src.model_traj.config import load_config, path_from_config
from src.model_traj.flat_bezier_mk2 import generate_trajectory


# Run this module as a script with the default parameters.
# Useful for regenerating reference files from the command line.
def main() -> None:
    cfg = load_config()
    output_dir = path_from_config(cfg, "reference_dir")
    generate_trajectory(
        output_dir=output_dir,
        generate_plots=bool(cfg.get("trajectory", {}).get("generate_plots", True)),
    )


if __name__ == "__main__":
    main()
