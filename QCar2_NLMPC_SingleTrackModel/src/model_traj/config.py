from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


# Return the project root folder.
def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


# Load the YAML configuration file.
def load_config(config_path: str | Path | None = None) -> dict[str, Any]:
    root = project_root()
    path = Path(config_path) if config_path is not None else root / "config" / "qcar2_nlmpc.yaml"
    if not path.is_absolute():
        path = root / path
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg or {}


# Convert a project-relative path into an absolute path.
def resolve_project_path(path_value: str | Path) -> Path:
    path = Path(path_value)
    return path if path.is_absolute() else project_root() / path


# Read one path from the YAML paths section.
def path_from_config(cfg: dict[str, Any], key: str) -> Path:
    return resolve_project_path(cfg["paths"][key])
