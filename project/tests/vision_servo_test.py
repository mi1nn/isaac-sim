"""Geometry used by the MEP capture / peg-in-hole vision servo.

These are the only parts of `scripts/ros2/` that can be checked without Isaac Sim
and a running ROS graph, so they are the parts worth unit testing: everything else
in that directory is a servo loop whose correctness only shows up in the simulator.

The module under test deliberately depends on nothing but NumPy, which is what lets
this file run under the plain system interpreter alongside the ROS 2 nodes.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1].joinpath("scripts", "ros2")))

from vision_servo import (  # noqa: E402
    angle_between,
    clamp_norm,
    fit_circle_3d,
    fit_plane_ransac,
    near_face_points,
    normalize,
)


def _axis_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle between two undirected axes.

    A plane fit pins down the normal's line, not which way along it points; only
    `orient_towards` resolves that. Tests that do not pass it must compare axes.
    """
    angle = np.degrees(angle_between(a, b))
    return min(angle, 180.0 - angle)


def _orthonormal_basis(normal: np.ndarray):
    n = normalize(np.asarray(normal, dtype=float))
    seed = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = normalize(np.cross(n, seed))
    return u, np.cross(n, u)


def _disc(center, normal, radius, n_points, rng, inner_radius=0.0):
    u, v = _orthonormal_basis(normal)
    theta = rng.uniform(0.0, 2.0 * np.pi, n_points)
    r = np.sqrt(rng.uniform(inner_radius**2, radius**2, n_points))
    return np.asarray(center) + np.outer(r * np.cos(theta), u) + np.outer(
        r * np.sin(theta), v
    )


## fit_plane_ransac ##


def test_fit_plane_recovers_a_tilted_plane():
    rng = np.random.default_rng(0)
    normal = normalize(np.array([0.3, -0.5, 1.0]))
    center = np.array([0.4, -1.2, 2.5])
    points = _disc(center, normal, 0.09, 400, rng)

    centroid, fitted = fit_plane_ransac(points, rng=rng)

    assert _axis_angle_deg(fitted, normal) < 0.5
    assert np.linalg.norm(centroid - center) < 5e-3


def test_fit_plane_ignores_outliers():
    """The side wall of the marker puck shows up as points off the front face."""
    rng = np.random.default_rng(1)
    normal = np.array([0.0, 0.0, 1.0])
    points = _disc(np.zeros(3), normal, 0.09, 300, rng)
    outliers = rng.uniform(-0.2, 0.2, (90, 3)) + np.array([0.0, 0.0, -0.15])

    _, fitted = fit_plane_ransac(
        np.vstack([points, outliers]), inlier_tol=5e-3, rng=rng
    )

    assert _axis_angle_deg(fitted, normal) < 1.0


def test_fit_plane_orients_the_normal_towards_a_reference_point():
    rng = np.random.default_rng(2)
    points = _disc(np.array([0.0, 0.0, 2.0]), np.array([0.0, 0.0, 1.0]), 0.09, 200, rng)

    # The camera sits at the origin, so the visible face points back at it (-Z)
    _, fitted = fit_plane_ransac(points, orient_towards=np.zeros(3), rng=rng)

    assert fitted[2] < 0.0


def test_fit_plane_returns_none_when_there_are_too_few_points():
    assert fit_plane_ransac(np.zeros((2, 3))) is None


## fit_circle_3d ##


def test_fit_circle_recovers_a_tilted_ring():
    rng = np.random.default_rng(3)
    normal = normalize(np.array([-0.2, 0.9, 0.4]))
    center = np.array([1.0, 2.0, -0.5])
    radius = 0.52
    points = _disc(center, normal, radius, 600, rng, inner_radius=radius - 1e-4)

    fitted_center, fitted_normal, fitted_radius = fit_circle_3d(points)

    assert np.linalg.norm(fitted_center - center) < 5e-3
    assert _axis_angle_deg(fitted_normal, normal) < 1.0
    assert fitted_radius == pytest.approx(radius, abs=5e-3)


def test_fit_circle_on_an_annulus_returns_the_mid_radius():
    """The green ring has real width, so its cloud fills the band between two radii."""
    rng = np.random.default_rng(4)
    normal = np.array([0.0, 1.0, 0.0])
    center = np.array([0.0, 3.0, 0.0])
    points = _disc(center, normal, 0.60, 900, rng, inner_radius=0.42)

    fitted_center, _, fitted_radius = fit_circle_3d(points)

    assert np.linalg.norm(fitted_center - center) < 1e-2
    assert 0.42 < fitted_radius < 0.60


def test_fit_circle_sees_only_a_partial_arc():
    """Half the ring is enough: the far side can be out of frame or self-occluded."""
    rng = np.random.default_rng(5)
    center = np.array([0.0, 0.0, 4.0])
    radius = 0.5
    u, v = _orthonormal_basis(np.array([0.0, 0.0, 1.0]))
    theta = rng.uniform(-0.6 * np.pi, 0.6 * np.pi, 400)
    points = center + np.outer(radius * np.cos(theta), u) + np.outer(
        radius * np.sin(theta), v
    )

    fitted_center, _, fitted_radius = fit_circle_3d(points)

    assert np.linalg.norm(fitted_center - center) < 1e-2
    assert fitted_radius == pytest.approx(radius, abs=1e-2)


def test_fit_circle_returns_none_when_there_are_too_few_points():
    assert fit_circle_3d(np.zeros((3, 3))) is None


## near_face_points ##


def test_near_face_points_keeps_the_half_closest_to_the_camera():
    points = np.stack(
        [np.zeros(100), np.zeros(100), np.linspace(1.0, 2.0, 100)], axis=1
    )

    near = near_face_points(points, fraction=0.5)

    assert len(near) == 50
    assert near[:, 2].max() <= 1.51


def test_near_face_points_never_drops_below_the_minimum():
    points = np.stack([np.zeros(8), np.zeros(8), np.arange(8.0)], axis=1)

    assert len(near_face_points(points, fraction=0.1, min_points=6)) == 6


## small helpers ##


def test_clamp_norm_only_shortens():
    assert np.allclose(clamp_norm(np.array([3.0, 4.0, 0.0]), 5.0), [3.0, 4.0, 0.0])
    assert np.linalg.norm(clamp_norm(np.array([3.0, 4.0, 0.0]), 1.0)) == pytest.approx(
        1.0
    )


def test_angle_between_is_symmetric_and_handles_identical_vectors():
    a, b = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    assert angle_between(a, b) == pytest.approx(np.pi / 2)
    assert angle_between(a, a) == pytest.approx(0.0, abs=1e-9)


## detect_color_blob ##


def _green_rgb(shape, patches):
    """A black image with a green rectangle per (row0, row1, col0, col1)."""
    rgb = np.zeros((*shape, 3), dtype=np.uint8)
    for r0, r1, c0, c1 in patches:
        rgb[r0:r1, c0:c1] = (0, 255, 0)
    return rgb


HSV_LOWER, HSV_UPPER = [45, 120, 60], [75, 255, 255]


def test_detect_color_blob_picks_the_largest_component():
    from vision_servo import detect_color_blob

    rgb = _green_rgb((60, 60), [(5, 9, 5, 9), (20, 40, 20, 40)])

    mask = detect_color_blob(rgb, HSV_LOWER, HSV_UPPER)

    assert mask[30, 30] > 0  # the big patch
    assert mask[6, 6] == 0  # the small one


def test_detect_color_blob_rejects_components_beyond_max_depth():
    """The capture marker is small and near; the thruster ring is large and far.

    Both are the same green. Gating on depth only after the largest component has
    been chosen would throw away the whole detection instead of the wrong one, so
    the gate has to come first.
    """
    from vision_servo import detect_color_blob

    rgb = _green_rgb((60, 60), [(5, 9, 5, 9), (20, 40, 20, 40)])
    depth = np.full((60, 60), 17.0, dtype=np.float32)
    depth[5:9, 5:9] = 6.5  # the near marker

    mask = detect_color_blob(
        rgb, HSV_LOWER, HSV_UPPER, depth=depth, max_depth=12.0
    )

    assert mask[6, 6] > 0
    assert mask[30, 30] == 0


def test_detect_color_blob_returns_empty_when_nothing_is_green():
    from vision_servo import detect_color_blob

    assert not detect_color_blob(np.zeros((20, 20, 3), np.uint8), HSV_LOWER, HSV_UPPER).any()
