"""Scaled reference-orbit geometry and visual-only USD authoring helpers."""

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class OrbitReferenceCfg:
    """Visual reference circle in the world frame; it does not drive body motion."""

    enabled: bool = True
    center_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    normal: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    radius_m: float = 14.0
    samples: int = 128
    color_rgba: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0)
    width_m: float = 0.05

    def validate(self) -> None:
        """Reject geometry or display values that cannot form a visible circle."""

        _validated_geometry(self)
        color = np.asarray(self.color_rgba, dtype=float)
        if color.shape != (4,) or not np.all((0.0 <= color) & (color <= 1.0)):
            raise ValueError(
                f"orbit_reference.color_rgba must have 4 values in [0, 1], got {self.color_rgba}"
            )
        if float(self.width_m) <= 0.0:
            raise ValueError("orbit_reference.width_m must be positive")


def _validated_geometry(cfg: OrbitReferenceCfg) -> tuple[np.ndarray, np.ndarray]:
    center = np.asarray(cfg.center_m, dtype=float)
    normal = np.asarray(cfg.normal, dtype=float)
    if center.shape != (3,):
        raise ValueError(f"orbit_reference.center_m must have 3 components, got {cfg.center_m}")
    if normal.shape != (3,):
        raise ValueError(f"orbit_reference.normal must have 3 components, got {cfg.normal}")
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= 1e-9:
        raise ValueError("orbit_reference.normal must be non-zero")
    if float(cfg.radius_m) <= 0.0:
        raise ValueError("orbit_reference.radius_m must be positive")
    if int(cfg.samples) < 3:
        raise ValueError("orbit_reference.samples must be at least 3")
    return center, normal / normal_norm


def sample_reference_orbit(cfg: OrbitReferenceCfg) -> np.ndarray:
    """Return evenly spaced world-frame points on the configured reference circle."""

    center, normal = _validated_geometry(cfg)
    seed_axis = np.eye(3)[int(np.argmin(np.abs(normal)))]
    axis_u = np.cross(normal, seed_axis)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    theta = np.linspace(0.0, 2.0 * np.pi, int(cfg.samples), endpoint=False)
    return center + float(cfg.radius_m) * (
        np.cos(theta)[:, None] * axis_u + np.sin(theta)[:, None] * axis_v
    )


def spawn_reference_orbit(stage, prim_path: str, cfg: OrbitReferenceCfg):
    """Author a persistent, visual-only periodic USD curve for the reference orbit."""

    cfg.validate()
    from pxr import Gf, UsdGeom

    points = sample_reference_orbit(cfg)
    color = tuple(float(value) for value in cfg.color_rgba)
    curve = UsdGeom.BasisCurves.Define(stage, prim_path)
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateWrapAttr(UsdGeom.Tokens.periodic)
    curve.CreateCurveVertexCountsAttr([len(points)])
    curve.CreatePointsAttr([Gf.Vec3f(*(float(value) for value in point)) for point in points])
    curve.CreateWidthsAttr([float(cfg.width_m)])
    curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curve.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
    curve.CreateDisplayOpacityAttr([color[3]])
    return curve
