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
    # Phase 1 default: translation only, the configured angular velocity is not applied
    assert cfg.mep.motion_mode == "translation_only"
    assert np.array_equal(cfg.mep.angular_velocity_w(), np.zeros(3))
    assert 0.01 <= cfg.mep.linear_velocity_mps <= 0.02
    assert cfg.prediction.horizon_sec == 0.3
    assert cfg.apriltag.family == "tag36h11"
    assert abs(np.linalg.norm(cfg.mep.drift_direction) - 1.0) < 1e-9


def test_config_override(cfg):
    c = vision.load_vision_config(overrides=["mep.linear_velocity_mps=0.02", "prediction.horizon_sec=0.5"])
    assert c.mep.linear_velocity_mps == 0.02 and c.prediction.horizon_sec == 0.5
    with pytest.raises(KeyError):
        vision.load_vision_config(overrides=["mep.nope=1"])
    with pytest.raises(KeyError):  # replaced by mep.angular_velocity_rad_s
        vision.load_vision_config(overrides=["mep.angular_velocity_deg_s=1.0"])
    with pytest.raises(ValueError):
        vision.load_vision_config(overrides=["mep.motion_mode=rotation_only"])
    with pytest.raises(ValueError):
        vision.load_vision_config(overrides=["mep.angular_velocity_rad_s=[0.1, 0.2]"])
    with pytest.raises(ValueError):  # above prediction.max_plausible_angular_rate_rad_s
        vision.load_vision_config(overrides=["mep.motion_mode=six_dof", "mep.angular_velocity_rad_s=[0.3, 0.0, 0.0]"])


def test_config_six_dof():
    c = vision.load_vision_config(overrides=["mep.motion_mode=six_dof", "mep.angular_velocity_rad_s=[0.01, -0.02, 0.03]"])
    assert c.mep.six_dof
    assert np.allclose(c.mep.angular_velocity_w(), [0.01, -0.02, 0.03])
    assert np.allclose(c.mep.linear_velocity_w(), np.asarray(c.mep.drift_direction) * c.mep.linear_velocity_mps)
    t = vision.load_vision_config(overrides=["mep.angular_velocity_rad_s=[0.01, -0.02, 0.03]"])
    assert not t.mep.six_dof and np.array_equal(t.mep.angular_velocity_w(), np.zeros(3))


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


#############
### 6-DoF ###
#############


def _rigid_motion(t, com0, v, w_w, r0, com_in_body):
    """Exact torque-free motion with constant world-frame w: the body frame M at time t."""
    r = vision.so3_exp(np.asarray(w_w) * t) @ r0
    com = com0 + np.asarray(v) * t
    return Frame(com - r @ com_in_body, r)


def test_so3_exp_log_and_body_world_equivalence():
    rng = np.random.default_rng(7)
    for _ in range(20):
        w = rng.normal(0.0, 1.0, 3)
        assert np.allclose(vision.so3_log(vision.so3_exp(w * 0.5)), w * 0.5, atol=1e-9)
        r = vision.so3_exp(rng.normal(0.0, 1.0, 3))
        h = 0.3
        # R Exp([R^T w] h) == Exp([w] h) R  (body-frame form used by the predictor)
        assert np.allclose(r @ vision.so3_exp((r.T @ w) * h), vision.so3_exp(w * h) @ r, atol=1e-12)


def test_euler_accumulation_is_not_used():
    """Combined rotation: the SO(3) integral differs from adding roll/pitch/yaw rates."""
    w = np.array([0.5, -0.4, 0.6])  # rad/s, world
    r0 = _rot([0.2, 1.0, -0.3], 50.0)
    h = 1.0
    exact = vision.so3_exp(w * h) @ r0
    p = vision.ConstantTwistPredictor(vision.PredictionCfg(horizon_sec=h, velocity_window_sec=2.0, orientation_window_sec=2.0,
                                                           min_samples=10, max_plausible_angular_rate_rad_s=2.0), Frame.identity())
    for i in range(21):
        t = 0.1 * i
        p.add(vision.Estimate(t, Frame(np.zeros(3), vision.so3_exp(w * t) @ r0)))
    pred = p.predict(0.0, horizon=h)  # from t = 0 to t = 1
    assert np.allclose(p.angular_velocity(), w, atol=1e-6)
    assert math.degrees(frames.rotation_angle(pred.rot, exact)) < 1e-4
    rpy0 = np.radians(vision.rotmat_to_rpy_deg(r0))
    rpy1 = np.radians(vision.rotmat_to_rpy_deg(vision.so3_exp(w * 0.1) @ r0))
    euler = rpy0 + (rpy1 - rpy0) / 0.1 * h
    r_euler = _rot([0, 0, 1], math.degrees(euler[2])) @ _rot([0, 1, 0], math.degrees(euler[1])) @ _rot([1, 0, 0], math.degrees(euler[0]))
    assert math.degrees(frames.rotation_angle(r_euler, exact)) > 1.0


@pytest.mark.parametrize("reference", ["tag", "mep_com"])
@pytest.mark.parametrize("noise", [False, True])
def test_constant_twist_prediction_offset_point(noise, reference):
    """Cylinder_01 far from the centre of mass moves on an arc: the SE(3) prediction
    (reference frame + fixed T_ref_Y) follows it, the translation-only one does not.

    reference "mep_com": exact (the COM moves in a straight line). reference "tag": the
    tags orbit the COM, the constant-velocity fit is biased by about
    a (T^2/12 + T h/2 + h^2/2), a = |w|^2 r (documented model limit)."""
    rng = np.random.default_rng(11)
    v = np.array([0.0, -0.008, 0.006])
    w = np.array([0.05, -0.04, 0.06])  # 5 deg/s: large, to make the arc visible
    r0 = _rot([0.3, -0.2, 1.0], 40.0)
    com0 = np.array([5.0, -2.7, 3.1])
    com_in_body = np.array([0.1, -0.2, 0.05])
    t_m_t = Frame(np.array([0.0, 0.0, 1.8]), _rot([1, 0, 0], 15.0))  # tags 1.8 m from the body origin
    t_t_y = Frame(np.array([0.0, 0.0, -0.045]), np.eye(3))
    cfg = vision.PredictionCfg(horizon_sec=0.3, velocity_window_sec=2.0, orientation_window_sec=5.0, min_samples=10)
    # same construction as VisionCaptureDemo
    t_ref_y = Frame(com_in_body, np.eye(3)).inv() @ t_m_t @ t_t_y if reference == "mep_com" else t_t_y
    six = vision.ConstantTwistPredictor(cfg, t_ref_y)
    lin = vision.LinearDriftPredictor(cfg)
    t_now = 6.0
    for i in range(int(t_now / 0.1) + 1):
        t = 0.1 * i
        y = _rigid_motion(t, com0, v, w, r0, com_in_body) @ t_m_t @ t_t_y
        if noise:  # PnP-like noise: 0.5 mm, 0.1 deg
            y = Frame(y.pos + rng.normal(0.0, 5e-4, 3), y.rot @ vision.so3_exp(np.radians(rng.normal(0.0, 0.1, 3))))
        six.add(vision.Estimate(t, y))
        lin.add(vision.Estimate(t, y))
    h, big_t = cfg.horizon_sec, cfg.velocity_window_sec
    truth_now = _rigid_motion(t_now, com0, v, w, r0, com_in_body) @ t_m_t @ t_t_y
    truth = _rigid_motion(t_now + h, com0, v, w, r0, com_in_body) @ t_m_t @ t_t_y
    assert np.linalg.norm(six.angular_velocity() - w) < (2e-3 if noise else 1e-6)
    pred = six.predict(t_now)
    pe, ae = vision.pose_errors(truth, pred)
    r_tag = float(np.linalg.norm((t_m_t.pos - com_in_body) - np.dot(t_m_t.pos - com_in_body, w) * w / np.dot(w, w)))
    bias = np.dot(w, w) * r_tag * (big_t ** 2 / 12 + big_t * h / 2 + h ** 2 / 2) if reference == "tag" else 0.0
    tol_p = 1.3 * bias + (2e-3 if noise else 1e-6)
    tol_a = 0.1 if noise else 1e-4
    assert pe < tol_p and ae < tol_a, (pe, ae, bias)
    if reference == "tag" and not noise:
        assert pe > 0.5 * bias  # the bias is real, not an artefact of the test
    assert vision.pose_errors(truth_now, six.estimate(t_now))[0] < tol_p
    # rigid velocity field: velocity of Cylinder_01 = numerical derivative of the truth
    dt = 1e-4
    v_y = ((_rigid_motion(t_now + dt, com0, v, w, r0, com_in_body) @ t_m_t @ t_t_y).pos - truth_now.pos) / dt
    v_tol = (np.dot(w, w) * r_tag * big_t if reference == "tag" else 0.0) + (3e-3 if noise else 1e-5)
    assert np.linalg.norm(six.velocity_at(truth_now.pos, t_now) - v_y) < v_tol
    # the translation-only predictor keeps the orientation fixed: its error is the
    # rotation over (window mean -> t + h), far above the 6-DoF prediction
    assert vision.pose_errors(truth, lin.predict(t_now))[1] > 5.0 * max(ae, 0.02)


def test_tag_reference_bias_small_at_configured_rate(cfg):
    """The configured six_dof rate (|w| ~ 0.5 deg/s) with the tags 2 m from the COM: the
    `tag` reference bias stays far below the capture tolerances (< 0.2 mm)."""
    w = np.asarray(vision.load_vision_config(overrides=["mep.motion_mode=six_dof"]).mep.angular_velocity_w())
    com_in_body = np.zeros(3)
    t_m_t = Frame(np.array([0.0, 2.0, 0.0]), _rot([1, 0, 0], -90.0))
    t_t_y = Frame(np.array([0.0, 0.0, -0.045]), np.eye(3))
    pcfg = cfg.prediction
    p = vision.ConstantTwistPredictor(pcfg, t_t_y)
    t_now = 8.0
    motion = lambda t: _rigid_motion(t, np.array([5.0, -2.7, 3.1]), cfg.mep.linear_velocity_w(), w, np.eye(3), com_in_body) @ t_m_t @ t_t_y
    for i in range(int(t_now / 0.1) + 1):
        p.add(vision.Estimate(0.1 * i, motion(0.1 * i)))
    pe, ae = vision.pose_errors(motion(t_now + pcfg.horizon_sec), p.predict(t_now))
    assert pe < 2e-4 and ae < 1e-4, (pe, ae)


def test_constant_twist_rejects_implausible_rate():
    cfg = vision.PredictionCfg(min_samples=5, max_plausible_angular_rate_rad_s=0.1)
    p = vision.ConstantTwistPredictor(cfg, Frame.identity())
    for i in range(10):
        p.add(vision.Estimate(0.1 * i, Frame(np.zeros(3), vision.so3_exp(np.array([0.0, 0.0, 0.5 * 0.1 * i])))))
    assert p.angular_velocity() is None and p.velocity() is None and p.predict(0.9) is None
    assert p.twist() == (None, None)
    assert p.estimate(0.9) is not None  # falls back to the latest measurement


def test_linear_predictor_interface_unchanged():
    """translation_only keeps the Phase 1 numbers through the shared interface."""
    p = vision.LinearDriftPredictor(vision.PredictionCfg(min_samples=10))
    r0 = _rot([0.2, 1.0, -0.3], 70.0)
    for i in range(20):
        p.add(vision.Estimate(0.1 * i, Frame(np.array([1.0, 2.0, 3.0]) + 0.01 * i, r0)))
    assert np.array_equal(p.orientation_at(123.0), p.orientation())
    assert np.array_equal(p.velocity_at(np.array([9.0, 9.0, 9.0]), 1.9), p.velocity())
    v, w = p.twist()
    assert np.array_equal(v, p.velocity()) and w is None


def test_six_dof_vision_pipeline(cfg, k):
    """Synthetic end-to-end: rotating + drifting constellation -> projected corners ->
    16-point PnP -> T_W_Y -> constant-twist prediction."""
    rng = np.random.default_rng(5)
    cam_w = Frame(np.zeros(3), np.eye(3))  # camera at the world origin
    v = np.array([0.004, -0.008, 0.006])
    w = np.array([0.005, -0.004, 0.006])  # config default, rad/s
    r0 = _rot([1, 0, 0], 180.0)  # constellation faces the camera
    com0 = np.array([0.02, -0.01, 1.0])
    com_in_body = np.array([0.0, 0.0, -0.8])  # rotation centre 0.8 m behind the tags
    t_t_y = Frame(np.array([0.0, 0.0, -0.045]), np.eye(3))
    pcfg = vision.PredictionCfg(horizon_sec=0.3, velocity_window_sec=2.0, orientation_window_sec=5.0, min_samples=10)
    pred = vision.ConstantTwistPredictor(pcfg, t_t_y)
    t_end = 8.0
    for i in range(int(t_end / 0.1) + 1):
        t = 0.1 * i
        t_w_t = _rigid_motion(t, com0, v, w, r0, com_in_body)
        pose = vision.estimate_constellation_pose(_synthetic(cfg, k, cam_w.inv() @ t_w_t, noise_px=0.1, seed=int(rng.integers(1 << 30))), cfg.apriltag, k)
        assert pose is not None
        pred.add(vision.Estimate(t, cam_w @ pose.t_c_t @ t_t_y))
    truth = _rigid_motion(t_end + 0.3, com0, v, w, r0, com_in_body) @ t_t_y
    pe, ae = vision.pose_errors(truth, pred.predict(t_end))
    assert np.linalg.norm(pred.angular_velocity() - w) < 2e-3, pred.angular_velocity()
    assert pe < 3e-3 and ae < 0.3, (pe, ae)


@pytest.mark.parametrize("mode", ["translation_only", "six_dof"])
def test_rendezvous_start_pose(mode):
    """Cylinder_01 starts at nominal - v t_r; translation_only is exactly the Phase 1 pose;
    rotating about Cylinder_01 for t_r with w brings the MEP back to the nominal pose."""
    c = vision.load_vision_config(overrides=[f"mep.motion_mode={mode}"])
    v, w, t_r = c.mep.linear_velocity_w(), c.mep.angular_velocity_w(), c.mep.rendezvous_time_s
    nominal = Frame(np.array([-0.80, 11.41, 8.41]), frames.quat_wxyz_to_rotmat((0.095277, -0.700659, 0.095277, -0.700659)))
    t_m_y = Frame(np.array([0.3, -1.2, 0.9]), _rot([1, 0, 0], 30.0))
    start = vision.rendezvous_start_pose(nominal, t_m_y, v, w, t_r)
    y_nom = (nominal @ t_m_y).pos
    assert np.abs((start @ t_m_y).pos - (y_nom - v * t_r)).max() < 1e-12
    if mode == "translation_only":
        assert np.array_equal(start.pos, nominal.pos - v * t_r) and np.array_equal(start.rot, nominal.rot)
    else:
        assert math.degrees(frames.rotation_angle(start.rot, nominal.rot)) == pytest.approx(math.degrees(np.linalg.norm(w) * t_r), rel=1e-9)
        # forward: drift the pivot, turn about it -> nominal
        q = vision.so3_exp(w * t_r)
        y_end = (start @ t_m_y).pos + v * t_r
        end = Frame(y_end + q @ (start.pos - (start @ t_m_y).pos), q @ start.rot)
        assert vision.pose_errors(nominal, end)[0] < 1e-12 and vision.pose_errors(nominal, end)[1] < 1e-5  # acos floor
