from __future__ import annotations

import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.model_traj.config import load_config, path_from_config


# Remove generated files while keeping the folder itself.
def _clean_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    for item in path.iterdir():
        if item.name == ".gitkeep":
            continue
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()


# Clean the generated OCP and simulation folders.
def main() -> None:
    cfg = load_config()
    for key in ("generated_ocp_dir", "generated_sim_dir"):
        path = path_from_config(cfg, key)
        _clean_dir(path)
        print(f"Cleaned {path}")


if __name__ == "__main__":
    main()
