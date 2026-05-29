from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import shutil

from src.model_traj.config import load_config, path_from_config


# Remove generated files inside one directory while keeping .gitkeep.
# Missing directories are created automatically.
def _clean_dir(path):
    path.mkdir(parents=True, exist_ok=True)
    for item in path.iterdir():
        if item.name == ".gitkeep":
            continue
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()


# Run this module as a script with the default parameters.
# Useful for regenerating reference files from the command line.
def main() -> None:
    cfg = load_config()
    for key in ("generated_ocp_dir", "generated_sim_dir"):
        path = path_from_config(cfg, key)
        _clean_dir(path)
        print(f"Cleaned {path}")


if __name__ == "__main__":
    main()
