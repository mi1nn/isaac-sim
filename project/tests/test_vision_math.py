"""Offline unit tests of the Phase 1 vision math (no Isaac Sim needed).

Run with the Isaac Sim Python (it has OpenCV):
    cd ~/space_robotics_bench/project && ~/isaac-sim/python.sh -m pytest tests/test_vision_math.py -q

The `debris_capture` package `__init__` imports Isaac Lab, so `frames.py` and
`vision.py` are loaded as members of a stand-in package instead.
"""

import importlib
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

_PKG_DIR = Path(__file__).resolve().parents[1].joinpath("srb", "tasks", "manipulation", "debris_capture")
_PKG = "_debris_capture_offline"
if _PKG not in sys.modules:
    pkg = types.ModuleType(_PKG)
    pkg.__path__ = [str(_PKG_DIR)]
    sys.modules[_PKG] = pkg
vision = importlib.import_module(f"{_PKG}.vision")
frames = importlib.import_module(f"{_PKG}.frames")
Frame = frames.Frame


def _rot(axis, deg):
    return vision.rotmat_from_rotvec(np.asarray(axis, dtype=float) / np.linalg.norm(axis) * math.radians(deg))


def _synthetic(cfg, k, t_c_t: Frame, noise_px=0.0, seed=0):
    rng = np.random.default_rng(seed)
    dets = {}
    for tid, corners in vision.tag_object_corners(cfg.apriltag).items():
        pc = (t_c_t.rot @ corners.T).T + t_c_t.pos
        uv = vision.project_points(pc, k) + rng.normal(0.0, noise_px, (4, 2))
        dets[tid] = vision.TagDetection(tid, uv)
    return dets


@pytest.fixture(scope="module")
def cfg():
    return vision.load_vision_config()


@pytest.fixture(scope="module")
def k(cfg):
    c = cfg.camera
    return vision.intrinsics_from_usd(c.focal_length_mm, c.horizontal_aperture_mm, c.width, c.height)


def test_config_defaults(cfg):
    assert cfg.mep.mass_kg == 3000.0
    assert cfg.mep.angular_velocity_deg_s == 0.0
    assert 0.01 <= cfg.mep.linear_velocity_mps <= 0.02
    assert cfg.prediction.horizon_sec == 0.3
    assert cfg.apriltag.family == "tag36h11"
    assert abs(np.linalg.norm(cfg.mep.drift_direction) - 1.0) < 1e-9


def test_config_override(cfg):
    c = vision.load_vision_config(overrides=["mep.linear_velocity_mps=0.02", "prediction.horizon_sec=0.5"])
    assert c.mep.linear_velocity_mps == 0.02 and c.prediction.horizon_sec == 0.5
    with pytest.raises(KeyError):
        vision.load_vision_config(overrides=["mep.nope=1"])
    with pytest.raises(ValueError):
        vision.load_vision_config(overrides=["mep.angular_velocity_deg_s=1.0"])


def test_intrinsics_match_fov(cfg, k):
    hfov = 2.0 * math.degrees(math.atan(cfg.camera.width / 2.0 / k[0, 0]))
    assert abs(hfov - cfg.camera.horizontal_fov_deg) < 1e-9


def test_object_points_layout(cfg):
    corners = vision.tag_object_corners(cfg.apriltag)
    allp = np.vstack(list(corners.values()))
    assert allp.shape == (16, 3)
    # Constellation centre (mean of all tag centres) is the frame origin
    assert np.allclose(np.mean([c.mean(axis=0) for c in corners.values()], axis=0), 0.0)
    ok, detail = vision.check_constellation_layout(cfg.apriltag, keep_out_radius=cfg.capture.cylinder_radius_m, face_half_extent=1.04)
    assert ok, detail


@pytest.mark.parametrize("dist", [0.45, 1.0, 1.9])
def test_pnp_recovers_pose(cfg, k, dist):
    t_true = Frame(np.array([0.02, -0.01, dist]), _rot([0.3, 1.0, 0.2], 180.0 - 4.0))
    # the constellation must face the camera: +Z_T towards the camera
    assert t_true.rot[:, 2] @ (-t_true.pos) > 0
    est = vision.estimate_constellation_pose(_synthetic(cfg, k, t_true, noise_px=0.1), cfg.apriltag, k)
    assert est is not None and est.num_tags == 4 and est.num_points == 16
    pe, ae = vision.pose_errors(t_true, est.t_c_t)
    assert pe < 0.002 * dist / 0.45 and ae < 0.3, (pe, ae)


def test_pnp_requires_constellation(cfg, k):
    t_true = Frame(np.array([0.0, 0.0, 1.0]), _rot([1, 0, 0], 180.0))
    dets = _synthetic(cfg, k, t_true)
    dets.pop(cfg.apriltag.ids[2])
    assert vision.estimate_constellation_pose(dets, cfg.apriltag, k) is None  # 3 of 4 tags


def test_pnp_rejects_outlier_corner(cfg, k):
    t_true = Frame(np.array([0.0, 0.0, 1.0]), _rot([1, 0, 0], 180.0))
    dets = _synthetic(cfg, k, t_true)
    dets[cfg.apriltag.ids[0]].corners[1] += np.array([25.0, -18.0])
    est = vision.estimate_constellation_pose(dets, cfg.apriltag, k)
    assert est is not None and est.num_inliers == 15
    pe, ae = vision.pose_errors(t_true, est.t_c_t)
    assert pe < 1e-3 and ae < 0.1


def test_detector_on_rendered_texture(cfg, tmp_path):
    """Generated texture -> detector: IDs and corner order (TL, TR, BR, BL)."""
    import cv2

    tcfg = cfg.apriltag
    img = cv2.imread(str(vision.make_tag_texture(tcfg, tcfg.ids[1], tmp_path / "t.png")), cv2.IMREAD_GRAYSCALE)
    img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
    det = vision.AprilTagDetector(tcfg).detect(img)
    assert list(det) == [tcfg.ids[1]]
    c = det[tcfg.ids[1]].corners
    q = tcfg.quiet_zone_cells * tcfg.texture_px_per_cell + 50
    w = 8 * tcfg.texture_px_per_cell
    # AprilTag refinement: continuous image coordinates (pixel centres at +0.5)
    expected = np.array([[q, q], [q + w, q], [q + w, q + w], [q, q + w]], dtype=float)
    assert np.abs(c - expected).max() < 1.0, c


def test_linear_prediction():
    p = vision.LinearDriftPredictor(vision.PredictionCfg(horizon_sec=0.3, velocity_window_sec=2.0, min_samples=10))
    v = np.array([0.0, -0.008, 0.006])
    rng = np.random.default_rng(1)
    for i in range(40):
        t = 0.1 * i
        p.add(vision.Estimate(t, Frame(np.array([5.0, -2.7, 3.1]) + v * t + rng.normal(0, 3e-4, 3), np.eye(3))))
    assert np.linalg.norm(p.velocity() - v) < 1.5e-3
    pred = p.predict(3.9)
    assert np.linalg.norm(pred.pos - (np.array([5.0, -2.7, 3.1]) + v * 4.2)) < 1e-3
    assert np.linalg.norm(p.angular_velocity()) < 1e-9


def test_angle_helpers():
    assert vision.wrap_deg(179.0 - (-179.0)) == pytest.approx(-2.0)
    assert vision.wrap_deg(-190.0) == pytest.approx(170.0)
    r = _rot([0, 0, 1], 179.0)
    r2 = _rot([0, 0, 1], -179.0)
    assert vision.pose_errors(Frame(np.zeros(3), r), Frame(np.zeros(3), r2))[1] == pytest.approx(2.0, abs=1e-6)
    rpy = vision.rotmat_to_rpy_deg(_rot([0, 0, 1], 30.0) @ _rot([0, 1, 0], 20.0) @ _rot([1, 0, 0], 10.0))
    assert np.allclose(rpy, (10.0, 20.0, 30.0), atol=1e-6)


def test_downsample_keeps_geometry(cfg, k):
    """A point rendered at u in the 2x image lands at u/2 with K/2 (continuous coords)."""
    k2 = k.copy()
    k2[:2, :] *= 2.0
    assert np.allclose(vision.scale_intrinsics(k2, 2), k)
    img = np.zeros((8, 8), dtype=np.uint8)
    img[:, 4:] = 255  # edge at u = 4 in the large image
    small = vision.downsample(img, 2)
    assert small.shape == (4, 4) and (small[:, 2:] == 255).all() and (small[:, :2] == 0).all()  # edge at u = 2


def test_orientation_window_mean():
    """Zero-mean tilt noise on a constant orientation averages out."""
    p = vision.LinearDriftPredictor(vision.PredictionCfg(velocity_window_sec=2.0, min_samples=10))
    r0 = _rot([0.2, 1.0, -0.3], 70.0)
    rng = np.random.default_rng(3)
    worst = 0.0
    for i in range(20):
        noise = vision.rotmat_from_rotvec(np.radians(np.array([rng.normal(0, 0.6), rng.normal(0, 0.6), 0.0])))
        p.add(vision.Estimate(0.1 * i, Frame(np.zeros(3), r0 @ noise)))
        worst = max(worst, vision.pose_errors(Frame(np.zeros(3), r0), Frame(np.zeros(3), r0 @ noise))[1])
    est = p.estimate(1.9)
    err = vision.pose_errors(Frame(np.zeros(3), r0), est)[1]
    assert np.allclose(est.rot.T @ est.rot, np.eye(3), atol=1e-9) and np.linalg.det(est.rot) > 0
    assert err < 0.4 < worst, (err, worst)
