"""YAML loader for `ProbeDockCfg`, same `section.key=value` override style as Phase 1."""

from dataclasses import fields, is_dataclass
from pathlib import Path
from typing import Optional, Sequence

from .gps_link import GpsLinkCfg
from .probe_dock_demo import ProbeDockCfg

PROJECT_DIR = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG_PATH = PROJECT_DIR.joinpath("config", "probe_dock.yaml")


def _apply(obj, data: dict, where: str = ""):
    known = {f.name for f in fields(obj)} if is_dataclass(obj) else set()
    for key, value in data.items():
        if key not in known:
            raise KeyError(f"Unknown probe-dock config key '{where}{key}'")
        current = getattr(obj, key)
        if isinstance(current, dict):
            if not isinstance(value, dict):
                raise TypeError(f"'{where}{key}' must be a mapping")
            current.update(value)
        elif is_dataclass(current):
            if not isinstance(value, dict):
                raise TypeError(f"'{where}{key}' must be a mapping")
            _apply(current, value, f"{where}{key}.")
        else:
            setattr(obj, key, value)


def load_probe_dock_config(path: Optional[str | Path] = None, overrides: Sequence[str] = ()) -> ProbeDockCfg:
    import yaml

    cfg = ProbeDockCfg(gps=GpsLinkCfg())
    path = Path(path) if path else DEFAULT_CONFIG_PATH
    with open(path) as f:
        _apply(cfg, yaml.safe_load(f) or {})
    for item in overrides:
        key, _, raw = item.partition("=")
        section, _, name = key.strip().partition(".")
        if name:
            _apply(cfg, {section: {name: yaml.safe_load(raw)}})
        else:
            _apply(cfg, {section.strip(): yaml.safe_load(raw)})
    if cfg.approach_depth_m >= 0.0:
        raise ValueError(f"approach_depth_m must be outside the bore (negative), got {cfg.approach_depth_m}")
    if cfg.dock_depth_m <= 0.0:
        raise ValueError(f"dock_depth_m must be positive, got {cfg.dock_depth_m}")
    return cfg
