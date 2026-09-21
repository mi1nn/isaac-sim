"""Offline tests for the scaled reference-orbit geometry."""

import importlib
import sys
import types
from pathlib import Path

import numpy as np
import pytest

_PKG_DIR = Path(__file__).resolve().parents[1].joinpath(
    "srb", "tasks", "manipulation", "debris_capture"
)
_PKG = "_debris_capture_orbit_offline"
if _PKG not in sys.modules:
    pkg = types.ModuleType(_PKG)
    pkg.__path__ = [str(_PKG_DIR)]
    sys.modules[_PKG] = pkg

orbit = importlib.import_module(f"{_PKG}.orbit_reference")
vision = importlib.import_module(f"{_PKG}.vision")


def test_samples_circle_in_requested_plane():
    cfg = orbit.OrbitReferenceCfg(
        center_m=(1.0, 2.0, 3.0),
        normal=(2.0, 0.0, 0.0),
        radius_m=14.0,
        samples=128,
    )

    points = orbit.sample_reference_orbit(cfg)

    assert points.shape == (128, 3)
    assert np.max(np.abs(points[:, 0] - 1.0)) < 1e-10
    radii = np.linalg.norm(points[:, 1:] - np.array([2.0, 3.0]), axis=1)
    assert np.max(np.abs(radii - 14.0)) < 1e-10


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"radius_m": 0.0}, "radius_m"),
        ({"samples": 2}, "samples"),
        ({"normal": (0.0, 0.0, 0.0)}, "normal"),
    ],
)
def test_rejects_invalid_geometry(changes, message):
    values = {
        "center_m": (0.0, 0.0, 0.0),
        "normal": (1.0, 0.0, 0.0),
        "radius_m": 14.0,
        "samples": 128,
    }
    values.update(changes)

    with pytest.raises(ValueError, match=message):
        orbit.sample_reference_orbit(orbit.OrbitReferenceCfg(**values))


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"width_m": 0.0}, "width_m"),
        ({"color_rgba": (1.0, 0.0, 0.0)}, "color_rgba"),
        ({"color_rgba": (1.1, 0.0, 0.0, 1.0)}, "color_rgba"),
    ],
)
def test_rejects_invalid_display_values(changes, message):
    cfg = orbit.OrbitReferenceCfg(**changes)

    with pytest.raises(ValueError, match=message):
        cfg.validate()


def test_default_reference_is_red_yz_circle():
    cfg = orbit.OrbitReferenceCfg()

    cfg.validate()

    assert cfg.center_m == (0.0, 0.0, 0.0)
    assert cfg.normal == (1.0, 0.0, 0.0)
    assert cfg.radius_m == 14.0
    assert cfg.samples == 128
    assert cfg.color_rgba == (1.0, 0.0, 0.0, 1.0)


def test_vision_config_exposes_reference_orbit():
    cfg = vision.VisionCaptureConfig()

    assert cfg.orbit_reference == orbit.OrbitReferenceCfg()
