"""Offline tests for the Ares1 probe -> thruster docking math (`probe_dock.py`).

No Isaac Sim needed:

    cd ~/space_robotics_bench/project && ~/isaac-sim/python.sh -m pytest tests/test_probe_dock.py -q
"""

import importlib
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

# The `debris_capture` package `__init__` imports Isaac Lab, so the pure-numpy modules
# are loaded as members of a stand-in package instead (as in `test_vision_math.py`).
_PKG_DIR = Path(__file__).resolve().parents[1].joinpath("srb", "tasks", "manipulation", "debris_capture")
_PKG = "_debris_capture_offline"
if _PKG not in sys.modules:
    pkg = types.ModuleType(_PKG)
    pkg.__path__ = [str(_PKG_DIR)]
    sys.modules[_PKG] = pkg
pd = importlib.import_module(f"{_PKG}.probe_dock")
_frames = importlib.import_module(f"{_PKG}.frames")
Frame, frame_from_axes = _frames.Frame, _frames.frame_from_axes
load_vision_config = importlib.import_module(f"{_PKG}.vision").load_vision_config


def rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def rot_z(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


@pytest.fixture
def dock():
    """A satellite dock frame in a deliberately non-axis-aligned world pose."""
    return frame_from_axes([3.1, -2.4, 5.7], [0.3, -0.8, 0.52], [0.0, 0.0, 1.0])


@pytest.fixture
def cfg():
    return pd.DockingVisionCfg()


### Relative pose ###


def test_perfectly_docked_probe_has_zero_error(dock):
    m = pd.dock_errors(Frame(dock.pos.copy(), dock.rot.copy()), dock)
    assert abs(m["axial"]) < 1e-12 and m["lateral"] < 1e-12
    assert m["axis_deg"] < 1e-9 and m["roll_deg"] < 1e-9
    # `orientation_deg` goes through acos(trace) and so is only sqrt-accurate near the
    # identity (~1e-6 deg); it is a reported quantity, never a gate.
    assert m["orientation_deg"] < 1e-4


def test_errors_are_measured_in_the_dock_frame_not_world(dock):
    """A tip 0.4 m *outside* along the docking axis, offset 30 mm sideways in the dock
    frame: axial must be -0.4 and lateral 0.03 whatever the world orientation is."""
    offset = -0.4 * dock.rot[:, 2] + 0.03 * dock.rot[:, 0]
    m = pd.dock_errors(Frame(dock.pos + offset, dock.rot.copy()), dock)
    assert m["axial"] == pytest.approx(-0.4)
    assert m["lateral"] == pytest.approx(0.03)
    assert m["e_x"] == pytest.approx(0.03) and abs(m["e_y"]) < 1e-12
    # Remaining insertion distance is the positive quantity the controller uses
    assert -m["axial"] == pytest.approx(0.4)


def test_axis_and_roll_errors_are_separated(dock):
    probe = Frame(dock.pos.copy(), dock.rot @ rot_x(2.0))  # tilt about the dock X axis
    m = pd.dock_errors(probe, dock)
    assert m["axis_deg"] == pytest.approx(2.0, abs=1e-6)
    assert m["roll_deg"] == pytest.approx(0.0, abs=1e-6)  # +X untouched by a rotation about X
    probe = Frame(dock.pos.copy(), dock.rot @ rot_z(3.0))  # pure roll about the docking axis
    m = pd.dock_errors(probe, dock)
    assert m["axis_deg"] == pytest.approx(0.0, abs=1e-6)
    assert m["roll_deg"] == pytest.approx(3.0, abs=1e-6)
    assert m["orientation_deg"] == pytest.approx(3.0, abs=1e-6)


def test_rpy_errors_are_display_only_and_match_a_known_rotation(dock):
    probe = Frame(dock.pos.copy(), dock.rot @ rot_z(5.0))
    r, p, y = pd.rpy_errors_deg(probe, dock)
    assert (abs(r), abs(p), y) == pytest.approx((0.0, 0.0, 5.0), abs=1e-6)


def test_insertion_depth_is_negative_outside_the_nozzle(dock):
    exit_w = Frame(dock.pos - 0.8 * dock.rot[:, 2], dock.rot.copy())  # dock point 0.8 m inside
    outside = Frame(exit_w.pos - 0.25 * dock.rot[:, 2], dock.rot.copy())
    inside = Frame(exit_w.pos + 0.3 * dock.rot[:, 2], dock.rot.copy())
    assert pd.insertion_depth(outside, exit_w) == pytest.approx(-0.25)
    assert pd.insertion_depth(inside, exit_w) == pytest.approx(0.3)


def test_tip_goal_lies_on_the_docking_axis(dock):
    g = pd.tip_goal(dock, -1.5)
    m = pd.dock_errors(g, dock)
    assert m["lateral"] < 1e-12 and m["axial"] == pytest.approx(-1.5)
    assert m["axis_deg"] < 1e-9 and m["roll_deg"] < 1e-9


### Speed profile ###


def test_approach_speed_has_three_zones_and_is_monotonic(cfg):
    assert pd.approach_speed(cfg, 5.0) == cfg.approach_speed_mps
    assert pd.approach_speed(cfg, cfg.slow_zone_m) == cfg.approach_speed_mps
    assert pd.approach_speed(cfg, cfg.insertion_zone_m) == cfg.insertion_speed_mps
    assert pd.approach_speed(cfg, 0.0) == cfg.insertion_speed_mps
    assert pd.approach_speed(cfg, -1.0) == cfg.insertion_speed_mps  # past the dock point
    mid = pd.approach_speed(cfg, 0.5 * (cfg.slow_zone_m + cfg.insertion_zone_m))
    assert cfg.insertion_speed_mps < mid < cfg.near_speed_mps
    speeds = [pd.approach_speed(cfg, r) for r in np.linspace(0.0, 1.0, 50)]
    assert all(b >= a - 1e-12 for a, b in zip(speeds, speeds[1:]))  # never speeds up on the way in


### Depth ###


def test_sample_depth_takes_the_median_at_the_principal_point():
    k = np.array([[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]])
    depth = np.full((48, 64), 9.0)
    depth[20:29, 28:37] = 2.0  # 9x9 patch centred on (32, 24)
    d, n = pd.sample_depth(depth, k, 9, 0.05, 12.0)
    assert d == pytest.approx(2.0) and n == 81


def test_sample_depth_rejects_inf_nan_and_out_of_range():
    k = np.array([[100.0, 0.0, 10.0], [0.0, 100.0, 10.0], [0.0, 0.0, 1.0]])
    depth = np.full((20, 20), np.inf)
    assert math.isnan(pd.sample_depth(depth, k, 5, 0.05, 12.0)[0])
    depth[:] = np.nan
    assert math.isnan(pd.sample_depth(depth, k, 5, 0.05, 12.0)[0])
    depth[:] = 0.0  # RTX writes 0 where nothing is hit
    assert math.isnan(pd.sample_depth(depth, k, 5, 0.05, 12.0)[0])
    depth[:] = 50.0  # beyond depth_max_m
    assert math.isnan(pd.sample_depth(depth, k, 5, 0.05, 12.0)[0])
    # A mix: only the valid readings count
    depth[:] = np.inf
    depth[8:13, 8:13] = 3.0
    d, n = pd.sample_depth(depth, k, 5, 0.05, 12.0)
    assert d == pytest.approx(3.0) and n == 25


def test_depth_to_dock_distance_matches_the_geometry(dock):
    """Camera 0.35 m behind the tip, back plate 0.1 m behind the dock point: a depth
    reading must reproduce the remaining insertion distance from the frames."""
    cam_to_tip, backstop = 0.35, 0.1
    remaining = 0.62
    probe = pd.tip_goal(dock, -remaining)
    depth = cam_to_tip + remaining + backstop  # what the camera would read on the plate
    assert pd.depth_to_dock_distance(depth, cam_to_tip, backstop) == pytest.approx(remaining)
    assert pd.depth_to_dock_distance(depth, cam_to_tip, backstop) == pytest.approx(-pd.dock_errors(probe, dock)["axial"])
    assert math.isnan(pd.depth_to_dock_distance(math.nan, cam_to_tip, backstop))


def test_depth_validity_gate(cfg):
    assert pd.depth_valid(cfg, 0.50, 0.52)[0]
    assert not pd.depth_valid(cfg, 0.50, 0.90)[0]  # disagrees by more than depth_agreement_m
    assert not pd.depth_valid(cfg, math.nan, 0.52)[0]
    off = pd.DockingVisionCfg(depth_enabled=False)
    assert not pd.depth_valid(off, 0.50, 0.50)[0]


### Conditions ###


def test_alignment_gate(cfg, dock):
    good = Frame(dock.pos - 0.5 * dock.rot[:, 2] + 0.002 * dock.rot[:, 0], dock.rot.copy())
    assert pd.alignment_ok(cfg, pd.dock_errors(good, dock))[0]
    off = Frame(dock.pos - 0.5 * dock.rot[:, 2] + 0.05 * dock.rot[:, 0], dock.rot.copy())
    ok, bad = pd.alignment_ok(cfg, pd.dock_errors(off, dock))
    assert not ok and "lateral" in bad[0]
    tilted = Frame(good.pos, dock.rot @ rot_x(3.0))
    ok, bad = pd.alignment_ok(cfg, pd.dock_errors(tilted, dock))
    assert not ok and any("axis" in b for b in bad)


def test_approach_abort_is_wider_than_the_alignment_gate(cfg, dock):
    """Between the two thresholds the approach continues but the gate no longer passes,
    so a re-align does not immediately abort the approach it just resumed."""
    between = 0.5 * (cfg.align_lateral_m + cfg.approach_abort_lateral_m)
    f = Frame(dock.pos - 0.3 * dock.rot[:, 2] + between * dock.rot[:, 0], dock.rot.copy())
    m = pd.dock_errors(f, dock)
    assert not pd.alignment_ok(cfg, m)[0]
    assert pd.approach_still_aligned(cfg, m)[0]
    far = Frame(dock.pos - 0.3 * dock.rot[:, 2] + 0.05 * dock.rot[:, 0], dock.rot.copy())
    assert not pd.approach_still_aligned(cfg, pd.dock_errors(far, dock))[0]


def test_wall_clearance():
    assert pd.wall_clearance(0.30, 0.12, 0.01) == pytest.approx(0.17)


def test_dock_ready_requires_every_condition(cfg, dock):
    at_dock = Frame(dock.pos.copy(), dock.rot.copy())
    m = pd.dock_errors(at_dock, dock)
    base = dict(m=m, rel_speed_mps=0.005, depth_ok=True, tracking_valid=True, clearance_m=0.15)
    ok, bad = pd.dock_ready(cfg, **base)
    assert ok and not bad
    # each condition on its own must be able to block the joint
    assert not pd.dock_ready(cfg, **{**base, "rel_speed_mps": 0.5})[0]
    assert not pd.dock_ready(cfg, **{**base, "rel_speed_mps": math.nan})[0]
    assert not pd.dock_ready(cfg, **{**base, "depth_ok": False})[0]
    assert not pd.dock_ready(cfg, **{**base, "tracking_valid": False})[0]
    assert not pd.dock_ready(cfg, **{**base, "clearance_m": 0.0})[0]
    assert not pd.dock_ready(cfg, **{**base, "clearance_m": math.nan})[0]
    far = pd.dock_errors(pd.tip_goal(dock, -0.05), dock)
    assert not pd.dock_ready(cfg, **{**base, "m": far})[0]
    tilted = pd.dock_errors(Frame(dock.pos.copy(), dock.rot @ rot_x(2.0)), dock)
    assert not pd.dock_ready(cfg, **{**base, "m": tilted})[0]


def test_dock_ready_without_depth_when_depth_is_not_required(cfg, dock):
    m = pd.dock_errors(Frame(dock.pos.copy(), dock.rot.copy()), dock)
    relaxed = pd.DockingVisionCfg(require_depth=False)
    assert pd.dock_ready(relaxed, m, 0.005, False, True, 0.15)[0]
    assert not pd.dock_ready(cfg, m, 0.005, False, True, 0.15)[0]


### Config ###


def test_docking_config_defaults_and_overrides():
    c = load_vision_config()
    assert c.docking.enabled is True and c.docking.require_depth is True
    c = load_vision_config(overrides=["docking.enabled=false", "docking.pre_dock_distance_m=1.5"])
    assert not c.docking.enabled and c.docking.pre_dock_distance_m == 1.5
    # the drift direction is normalised at load time, like mep.drift_direction
    c = load_vision_config(overrides=["docking.satellite_velocity_mps=0.004",
                                      "docking.satellite_drift_direction=[0.0, 2.0, 0.0]"])
    assert c.docking.satellite_drift_direction == [0.0, 1.0, 0.0]


def test_docking_config_rejects_impossible_values():
    for bad in (
        ["docking.satellite_velocity_mps=-1"],
        ["docking.satellite_velocity_mps=0.01", "docking.satellite_drift_direction=[0,0,0]"],
        ["docking.pre_dock_distance_m=0"],
        ["docking.insertion_speed_mps=1.0"],  # faster than the approach speed
        ["docking.approach_abort_lateral_m=0.001"],  # narrower than the alignment gate
        ["docking.depth_patch_px=4"],  # even
        ["docking.depth_min_m=0"],
        ["docking.insertion_zone_m=0.9"],  # larger than slow_zone_m
    ):
        with pytest.raises(ValueError):
            load_vision_config(overrides=bad)


def test_wall_clearance_is_unconstrained_outside_the_nozzle():
    """Outside the nozzle there is no wall on the approach axis (the lateral gate guards
    the rim), so the clearance must not report a huge negative number there."""
    assert math.isinf(pd.wall_clearance(0.30, 0.12, 5.5, insertion_depth_m=-1.0))
    assert pd.wall_clearance(0.30, 0.12, 0.01, insertion_depth_m=0.5) == pytest.approx(0.17)


def test_ramped_limits_acceleration_but_not_braking(cfg):
    dt = 1.0 / 150.0
    v = 0.0
    for _ in range(3):
        v = pd.ramped(cfg, v, 0.10, dt)
    assert v == pytest.approx(3 * cfg.accel_mps2 * dt)
    assert v < 0.10  # the ramp really limits the rise
    assert pd.ramped(cfg, 0.10, 0.0, dt) == 0.0  # a fail-safe can stop at once


def test_decelerated_never_exceeds_the_request_and_stops_short(cfg):
    assert pd.decelerated(cfg, 0.10, 10.0) == 0.10
    assert pd.decelerated(cfg, 0.10, 0.0) == cfg.creep_speed_mps
    near, far = pd.decelerated(cfg, 0.10, 0.2), pd.decelerated(cfg, 0.10, 2.0)
    assert near < far <= 0.10


def test_settled_needs_a_window_longer_than_a_swing_half_period(cfg):
    """A short window is satisfied at every turning point of a running oscillation, so
    `settled` must be given a window that spans the motion."""
    t = np.arange(0.0, 30.0, 0.1)
    swing = [(float(ti), np.array([0.025 * math.sin(2 * math.pi * ti / 20.0), 0.0, 0.0])) for ti in t]
    at_peak = [x for x in swing if x[0] <= 5.0]  # 5 s = the first quarter period, near the peak
    assert pd.settled(at_peak, 5.0, 2.0, 0.008)          # a 2 s window is fooled
    assert not pd.settled(swing, 30.0, 12.0, 0.008)      # a 12 s window is not
    still = [(float(ti), np.array([0.0, 0.0, 0.0])) for ti in t]
    assert pd.settled(still, 30.0, 12.0, 0.008)


def test_settled_rejects_a_history_shorter_than_the_window():
    hist = [(9.5, np.zeros(3)), (10.0, np.zeros(3))]
    assert not pd.settled(hist, 10.0, 12.0, 0.008)


def test_probe_camera_config_validation():
    c = load_vision_config()
    assert c.probe_camera.enabled and c.probe_camera.name == "cam_probe"
    assert c.probe_camera.focal_length_mm > 0.0
    for bad in (["probe_camera.width=4"], ["probe_camera.horizontal_fov_deg=200"],
                ["probe_camera.clipping_range_m=[0.0, 5.0]"], ["probe_camera.offset_from_tip_m=-0.1"]):
        with pytest.raises(ValueError):
            load_vision_config(overrides=bad)


def test_docking_speed_and_transport_validation():
    with pytest.raises(ValueError):
        load_vision_config(overrides=["docking.transport_speed_mps=0.01"])  # slower than the approach
    c = load_vision_config(overrides=["docking.transport_speed_mps=0.2"])
    assert c.docking.transport_speed_mps == 0.2


def test_depth_calibration_only_on_the_docking_axis(cfg):
    base = {"depth_raw": 3.0, "depth_pixels": 81.0, "lateral": 0.005, "axis_deg": 0.2}
    assert pd.depth_calibration_sample_ok(cfg, base)[0]
    # 41 mm off the axis passed the 50 mm alignment gate but put the ray on another
    # surface (constant 320 mm disagreement on the axis afterwards): rejected here
    ok, why = pd.depth_calibration_sample_ok(cfg, {**base, "lateral": 0.041})
    assert not ok and "lateral" in why
    assert not pd.depth_calibration_sample_ok(cfg, {**base, "axis_deg": cfg.depth_calibration_max_axis_deg + 0.1})[0]
    assert not pd.depth_calibration_sample_ok(cfg, {**base, "depth_raw": math.nan})[0]
    assert not pd.depth_calibration_sample_ok(cfg, {**base, "depth_pixels": 0.0})[0]
    # tighter than the (YAML) alignment gate, which let the 41 mm calibration through
    yaml_cfg = load_vision_config().docking
    assert yaml_cfg.depth_calibration_max_lateral_m < yaml_cfg.align_lateral_m


def test_depth_calibration_config_validation():
    with pytest.raises(ValueError):
        load_vision_config(overrides=["docking.depth_calibration_samples=0"])
    with pytest.raises(ValueError):
        load_vision_config(overrides=["docking.depth_calibration_max_lateral_m=0"])
