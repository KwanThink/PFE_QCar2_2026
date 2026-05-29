from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


# Return the project root directory.
# This keeps path handling independent of the current shell location.
def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


# Load the YAML configuration file.
# If no path is given, the default project config is used.
def load_config(config_path: str | Path | None = None) -> dict[str, Any]:
    root = project_root()
    path = Path(config_path) if config_path is not None else root / "config" / "qcar2_nlmpc.yaml"
    if not path.is_absolute():
        path = root / path
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg or {}


# Convert a config path into an absolute project path.
# Absolute paths are returned unchanged.
def resolve_project_path(path_value: str | Path) -> Path:
    path = Path(path_value)
    return path if path.is_absolute() else project_root() / path


# Read a path entry from the paths section of the config.
# The value is resolved relative to the project root when needed.
def path_from_config(cfg: dict[str, Any], key: str) -> Path:
    return resolve_project_path(cfg["paths"][key])
