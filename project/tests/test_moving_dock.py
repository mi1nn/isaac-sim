"""Offline tests for the moving-client rendezvous / stop / departure math (`moving_dock.py`).

No Isaac Sim needed:

    cd ~/space_robotics_bench/project && ~/isaac-sim/python.sh -m pytest tests/test_moving_dock.py -q
"""

import importlib
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

# The `debris_capture` package `__init__` imports Isaac Lab, so the pure-numpy modules
# are loaded as members of a stand-in package instead (as in `test_probe_dock.py`).
_PKG_DIR = Path(__file__).resolve().parents[1].joinpath("srb", "tasks", "manipulation", "debris_capture")
_PKG = "_debris_capture_offline"
if _PKG not in sys.modules:
    pkg = types.ModuleType(_PKG)
    pkg.__path__ = [str(_PKG_DIR)]
    sys.modules[_PKG] = pkg
md = importlib.import_module(f"{_PKG}.moving_dock")
load_vision_config = importlib.import_module(f"{_PKG}.vision").load_vision_config
TransitLeg = importlib.import_module(f"{_PKG}.mrv_approach").TransitLeg

DT = 1.0 / 60.0


def simulate_track(v_client, p_client0, p_mrv0, v_mrv0, d_ref, seconds, a_max=0.004, kp=0.15, v_corr=0.02):
    """Client drifting at constant velocity, MRV integrating the controller's command."""
    ctrl = md.MrvVelocityController(a_max, kp, v_corr)
    ctrl.reset(v_mrv0)
    p_c, p_m = np.asarray(p_client0, float), np.asarray(p_mrv0, float)
    v_c = np.asarray(v_client, float)
    hist = []
    for _ in range(int(seconds / DT)):
        v = ctrl.track(p_m, p_c, v_c, d_ref, DT)
        hist.append((np.linalg.norm(ctrl.a), np.linalg.norm(v - v_c), np.linalg.norm(ctrl.position_error(p_m, p_c, d_ref))))
        p_m = p_m + v * DT
        p_c = p_c + v_c * DT
    return ctrl, np.array(hist)


def test_relative_velocity_is_body_minus_client():
    v_client = np.array([0.02, 0.0, 0.0])
    # all three moving at 0.02 m/s in the world: relative velocity is zero
    assert np.allclose(md.relative([0.02, 0.0, 0.0], v_client), 0.0)
    assert np.allclose(md.relative([0.015, 0.0, 0.0], v_client), [-0.005, 0.0, 0.0])


def test_velocity_gate_ignores_world_speed():
    # moving at 20 mm/s in the world, 1 mm/s relative -> matched
    ok, bad = md.matched(np.array([0.001, 0, 0]), np.array([0.0, 0.001, 0]), 0.005, 0.005)
    assert ok and not bad
    ok, bad = md.matched(np.array([0.006, 0, 0]), np.array([0.0, 0.001, 0]), 0.005, 0.005)
    assert not ok and "MRV-client" in bad[0]
    ok, _ = md.matched(None, np.zeros(3), 0.005, 0.005)  # no estimate yet: never matched
    assert not ok


def test_mrv_controller_matches_velocity_and_station():
    d_ref = np.array([10.0, 3.0, 1.0])
    # MRV starts slower (0.015 vs 0.02 m/s) and 0.2 m behind its station
    ctrl, hist = simulate_track([0.02, 0, 0], d_ref + [0.2, 0, 0], [0.0, 0.0, 0.0], [0.015, 0, 0], d_ref, 60.0)
    assert hist[-1, 1] < 1e-4           # |v_mrv - v_client| -> 0
    assert hist[-1, 2] < 1e-3           # station error -> 0
    assert hist[:, 0].max() <= 0.004 + 1e-9  # acceleration limit respected every step


def test_mrv_controller_does_not_step_the_velocity():
    ctrl = md.MrvVelocityController(0.004, 0.15, 0.02)
    v = ctrl.toward([0.015, 0, 0], DT)
    assert np.linalg.norm(v) == pytest.approx(0.004 * DT)
    for _ in range(1000):
        v = ctrl.toward([0.015, 0, 0], DT)
    assert np.allclose(v, [0.015, 0, 0])


def test_position_error_sign_speeds_the_mrv_up_when_the_client_is_ahead():
    ctrl = md.MrvVelocityController(1.0, 0.2, 1.0)
    e = ctrl.position_error([0, 0, 0], [1.1, 0, 0], [1.0, 0, 0])
    assert np.allclose(e, [0.1, 0, 0])
    v = ctrl.track([0, 0, 0], [1.1, 0, 0], [0.02, 0, 0], [1.0, 0, 0], DT)
    assert v[0] > 0.0


def test_correction_speed_is_capped():
    ctrl = md.MrvVelocityController(10.0, 1.0, 0.02)
    ctrl.reset([0.02, 0, 0])
    v = ctrl.track([0, 0, 0], [5.0, 0, 0], [0.02, 0, 0], [0.0, 0, 0], 1.0)
    assert v[0] == pytest.approx(0.04)


def test_velocity_estimator_recovers_a_constant_velocity_and_waits_for_data():
    est = md.VelocityEstimator(0.5)
    assert est.velocity() is None
    v = np.array([0.02, -0.01, 0.005])
    for i in range(60):
        est.add(i * DT, np.array([1.0, 2.0, 3.0]) + v * i * DT)
    assert np.allclose(est.velocity(), v, atol=1e-9)
    assert est.t[-1] - est.t[0] <= 0.5 + 1e-9  # window kept


def test_stable_timer_resets_when_the_condition_breaks():
    tm = md.StableTimer()
    assert tm.update(True, 0.0) == 0.0
    assert tm.update(True, 0.6) == pytest.approx(0.6)
    assert tm.update(False, 0.7) == 0.0
    assert tm.update(True, 0.8) == 0.0
    assert tm.update(True, 1.9) == pytest.approx(1.1)


def test_bounded_accel_and_stop_law_bring_a_body_to_rest():
    v = np.array([0.02, 0.0, 0.0])
    peak = 0.0
    for _ in range(int(40.0 / DT)):
        a = md.bounded_accel(-v, 3.0, 0.002)
        peak = max(peak, float(np.linalg.norm(a)))
        v = v + a * DT
    assert peak <= 0.002 + 1e-12
    assert np.linalg.norm(v) < 0.001
    assert md.stopped([0.0005, 0.0002, 0.0], [1e-5], 0.001, 0.0005)
    assert not md.stopped([0.0005, None], [0.0], 0.001, 0.0005)


def test_separation_direction_is_opposite_the_client_drift():
    assert np.allclose(md.separation_direction([0.02, 0.0, 0.0]), [-1.0, 0.0, 0.0])
    assert np.allclose(md.separation_direction([0.0, 0.03, 0.04]), [0.0, -0.6, -0.8])
    # explicit override wins
    assert np.allclose(md.separation_direction([0.02, 0, 0], [0.0, 0.0, 2.0]), [0.0, 0.0, 1.0])
    with pytest.raises(ValueError):
        md.separation_direction([0.0, 0.0, 0.0])


def test_separation_leg_cruises_for_the_configured_time():
    v, a, cruise = 0.02, 0.004, 5.0
    length = md.separation_leg_length(v, a, cruise)
    leg = TransitLeg(np.zeros(3), length * np.array([-1.0, 0.0, 0.0]), v, a)
    at_speed, t = 0.0, 0.0
    while not leg.done(1e-9) and t < 100.0:
        leg.step(DT)
        t += DT
        if leg.v >= v - 1e-9:
            at_speed += DT
    assert leg.done(1e-9)
    assert at_speed == pytest.approx(cruise, abs=0.1)


def test_config_defaults_keep_the_existing_docking_unchanged():
    cfg = load_vision_config()
    assert cfg.client.release_enabled is False  # moving client is opt-in (--moving_dock)
    assert cfg.docking.satellite_velocity_mps == 0.0
    assert cfg.client.linear_velocity_mps == [0.02, 0.0, 0.0]
    assert cfg.separation.direction == []
    # demo defaults: no artificial chase gap, no cruise observation
    assert cfg.rendezvous.mrv_initial_velocity_mps == [] and not cfg.rendezvous.chase_enabled
    assert cfg.rendezvous.mrv_follow_during_release is True
    assert cfg.client.cruise_check_s == 0.0
    assert cfg.rendezvous.concurrent_docking is True  # no waiting states before the docking
    assert cfg.docking.settle_time_s == 0.0
    assert cfg.client.release_at_start is True  # the client drifts from t = 0 with --moving_dock
    assert cfg.post_docking.immediate_release is True  # release at once, stack stops, MRV departs


def test_spec_test_setup_is_still_available_through_overrides():
    cfg = load_vision_config(overrides=["docking.enabled=true", "client.release_enabled=true",
                                        "client.cruise_check_s=5.0",
                                        "rendezvous.mrv_initial_velocity_mps=[0.015,0.0,0.0]",
                                        "rendezvous.mrv_follow_during_release=false",
                                        "rendezvous.concurrent_docking=false"])
    assert cfg.rendezvous.concurrent_docking is False
    assert cfg.rendezvous.chase_enabled and cfg.rendezvous.mrv_initial_velocity_mps == [0.015, 0.0, 0.0]
    with pytest.raises(ValueError):
        load_vision_config(overrides=["client.cruise_check_s=-1"])


def test_following_from_the_release_leaves_no_station_gap():
    # client ramps 0 -> 0.02 m/s at 0.004 m/s^2 while the MRV holds its station
    ctrl = md.MrvVelocityController(0.004, 0.25, 0.02)
    p_c, p_m, v_c, d_ref = np.zeros(3), np.array([-10.0, 0, 0]), np.zeros(3), np.array([10.0, 0, 0])
    worst = 0.0
    for _ in range(int(15.0 / DT)):
        v_c = v_c + md.clip_norm(np.array([0.02, 0, 0]) - v_c, 0.004 * DT)
        v = ctrl.track(p_m, p_c, v_c, d_ref, DT)
        p_m, p_c = p_m + v * DT, p_c + v_c * DT
        worst = max(worst, float(np.linalg.norm(ctrl.position_error(p_m, p_c, d_ref))))
    assert worst < 0.01                      # never more than 10 mm behind
    assert np.linalg.norm(ctrl.v - v_c) < 1e-3


def test_config_moving_overrides_and_validation():
    cfg = load_vision_config(overrides=["docking.enabled=true", "client.release_enabled=true",
                                        "client.linear_velocity_mps=[0.0, 0.02, 0.0]",
                                        "rendezvous.mrv_initial_velocity_mps={x: 0.0, y: 0.015, z: 0.0}"])
    assert cfg.client.linear_velocity_mps == [0.0, 0.02, 0.0]
    assert cfg.rendezvous.mrv_initial_velocity_mps == [0.0, 0.015, 0.0]
    with pytest.raises(ValueError):  # the moving client needs the docking phase
        load_vision_config(overrides=["docking.enabled=false", "client.release_enabled=true"])
    with pytest.raises(ValueError):  # zero drift: no departure direction
        load_vision_config(overrides=["docking.enabled=true", "client.release_enabled=true", "client.linear_velocity_mps=[0,0,0]"])
    with pytest.raises(ValueError):
        load_vision_config(overrides=["rendezvous.mrv_max_accel_mps2=0"])
    with pytest.raises(ValueError):
        load_vision_config(overrides=["client.linear_velocity_mps=[0.02, 0.0]"])


def test_csv_has_every_requested_telemetry_column():
    required = ["timestamp", "state"]
    for body in ("client", "mep", "mrv"):
        for q in ("position", "velocity"):
            required += [f"{body}_{q}_{a}" for a in "xyz"]
    required += ["mep_client_relative_position", "mep_client_relative_velocity", "mrv_client_relative_position",
                 "mrv_client_relative_velocity", "docking_position_error", "docking_orientation_error",
                 "robot_mep_attached", "mep_client_docked", "separation_distance"]
    missing = [c for c in required if c not in md.CSV_COLUMNS]
    assert not missing, missing
    assert len(set(md.CSV_COLUMNS)) == len(md.CSV_COLUMNS)


def test_forced_insertion_corridor_is_wider_but_keeps_the_wall_limit():
    cfg = load_vision_config()
    rv, d = cfg.rendezvous, cfg.docking
    assert rv.forced_insertion is True
    assert rv.insertion_max_axis_deg >= d.approach_abort_axis_deg and rv.insertion_max_lateral_m >= d.approach_abort_lateral_m
    # inside the corridor: continues
    ok, _ = md.insertion_corridor_ok(0.10, 7.0, math.inf, rv.insertion_max_lateral_m, rv.insertion_max_axis_deg, d.min_wall_clearance_m)
    assert ok
    # ... but never closer to the nozzle wall than the minimum clearance
    ok, bad = md.insertion_corridor_ok(0.02, 1.0, 0.01, rv.insertion_max_lateral_m, rv.insertion_max_axis_deg, d.min_wall_clearance_m)
    assert not ok and "wall clearance" in bad[0]
    ok, bad = md.insertion_corridor_ok(0.02, rv.insertion_max_axis_deg + 1.0, math.inf, rv.insertion_max_lateral_m, rv.insertion_max_axis_deg, d.min_wall_clearance_m)
    assert not ok and "axis" in bad[0]
    with pytest.raises(ValueError):
        load_vision_config(overrides=["rendezvous.insertion_min_relative_speed_mps=0"])
