from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.model_traj.config import load_config, path_from_config
from src.model_traj.flat_bezier_single_track import generate_trajectory


# Regenerate the reference trajectory from the current YAML configuration.
def main() -> None:
    cfg = load_config()
    output_dir = path_from_config(cfg, "reference_dir")
    generate_trajectory(
        output_dir=output_dir,
        generate_plots=bool(cfg.get("trajectory", {}).get("generate_plots", True)),
    )


if __name__ == "__main__":
    main()
