"""Offline tests for the Client reference orbit and the orbit-return rules (`orbit_return.py`).

No Isaac Sim needed:

    cd ~/isaac_space/project && python3 -m pytest tests/test_orbit_return.py -q
"""

import importlib
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

# The `debris_capture` package `__init__` imports Isaac Lab, so the pure-numpy modules are
# loaded as members of a stand-in package instead (as in `test_probe_dock.py`).
_PKG_DIR = Path(__file__).resolve().parents[1].joinpath("srb", "tasks", "manipulation", "debris_capture")
_PKG = "_debris_capture_offline"
if _PKG not in sys.modules:
    pkg = types.ModuleType(_PKG)
    pkg.__path__ = [str(_PKG_DIR)]
    sys.modules[_PKG] = pkg
oc = importlib.import_module(f"{_PKG}.orbit_return")
load_vision_config = importlib.import_module(f"{_PKG}.vision").load_vision_config


def cfg(**kw):
    """A config for the generic geometry tests: the whole ring, radial offset, automatic nominal point.
    (The shipped defaults -- an 80 deg arc, straight-up offset, nominal angle 0 -- are tested below.)"""
    c = oc.OrbitReferenceCfg(enabled=True, arc_span_deg=360.0, nominal_angle_deg=None, client_offset_direction="radial_outward",
                             radius_m=4.0, semi_major_m=4.0, semi_minor_m=3.0)
    for k, v in kw.items():
        setattr(c, k, v)
    oc.validate_orbit_cfg(c)
    return c


def circle(center=(1.0, 2.0, 3.0), normal=(0.0, 0.0, 1.0), radius=4.0):
    return oc.OrbitReference.from_cfg(cfg(normal_w=list(normal), radius_m=radius), center)


def ellipse(center=(0.0, 0.0, 0.0), normal=(0.0, 0.0, 1.0), a=4.0, b=3.0):
    return oc.OrbitReference.from_cfg(cfg(shape="ellipse", normal_w=list(normal), semi_major_m=a, semi_minor_m=b), center)


########################
### Orbit generation ###
########################


def test_plane_basis_is_right_handed_orthonormal():
    for n in ([0, 0, 1], [1, 0, 0], [0.3, -0.5, 0.8], [0, 1, 0]):
        e1, e2, nn = oc.plane_basis(n)
        assert np.allclose([e1 @ e2, e1 @ nn, e2 @ nn], 0.0, atol=1e-12)
        assert np.allclose(np.cross(e1, e2), nn)
        assert np.allclose([np.linalg.norm(e1), np.linalg.norm(e2)], 1.0)


def test_circle_points_lie_on_the_circle_in_the_plane():
    o = circle(normal=(0.2, 0.3, 0.9))
    pts = o.points(128)
    assert pts.shape == (128, 3)
    r = np.linalg.norm(pts - o.center, axis=1)
    assert np.allclose(r, 4.0)
    assert np.allclose((pts - o.center) @ o.normal, 0.0, atol=1e-12)


def test_ellipse_points_satisfy_the_ellipse_equation():
    o = ellipse(center=(1, 2, 3), normal=(0.0, 1.0, 1.0))
    pts = o.points(256)
    d = pts - o.center
    x, y = d @ o.e1, d @ o.e2
    assert np.allclose((x / 4.0) ** 2 + (y / 3.0) ** 2, 1.0)
    assert np.allclose(d @ o.normal, 0.0, atol=1e-12)


def test_segments_below_128_are_rejected():
    with pytest.raises(ValueError, match="segments"):
        cfg(segments=64)


###################
### Distances ###
###################


def test_circle_distance_in_plane_and_out_of_plane():
    o = circle(center=(0, 0, 0))
    assert o.distance([4.5, 0, 0]) == pytest.approx(0.5)
    assert o.distance([3.0, 0, 0]) == pytest.approx(1.0)
    assert o.distance([4.0, 0, 0.3]) == pytest.approx(0.3)
    q, theta, d = o.closest([5.0, 5.0, 0.0])
    assert np.allclose(q, 4.0 * np.array([math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0]))
    assert d == pytest.approx(math.hypot(5, 5) - 4.0)
    # a point on the orbit is at distance 0
    assert o.distance(o.point_at(1.234)) == pytest.approx(0.0, abs=1e-12)


def test_ellipse_closest_point_matches_brute_force():
    o = ellipse(center=(0, 0, 0))
    rng = np.random.default_rng(1)
    dense = o.points(200000)
    for _ in range(25):
        p = rng.uniform(-6, 6, size=3)
        brute = float(np.linalg.norm(dense - p, axis=1).min())
        assert o.distance(p) == pytest.approx(brute, abs=1e-4)
        assert o.distance(p) <= brute + 1e-9  # the refined minimum is never worse than a sample


def test_ellipse_distance_on_the_axes():
    o = ellipse(center=(0, 0, 0))
    assert o.distance([5.0, 0, 0]) == pytest.approx(1.0)   # beyond the major vertex
    assert o.distance([0, 3.5, 0]) == pytest.approx(0.5)   # beyond the minor vertex


def test_centre_of_a_circle_has_a_defined_closest_point():
    q, theta, d = circle(center=(0, 0, 0)).closest([0, 0, 0])
    assert d == pytest.approx(4.0) and np.isfinite(q).all()


def test_outward_normal_is_radial_for_a_circle_and_unit_for_an_ellipse():
    o = circle(center=(0, 0, 0))
    for th in (0.0, 1.0, 4.0):
        assert np.allclose(o.outward_at(th), (o.point_at(th) - o.center) / 4.0)
    e = ellipse()
    for th in (0.3, 2.0, 5.0):
        n = e.outward_at(th)
        assert np.linalg.norm(n) == pytest.approx(1.0)
        # perpendicular to the tangent
        tangent = -e.a * math.sin(th) * e.e1 + e.b * math.cos(th) * e.e2
        assert n @ tangent == pytest.approx(0.0, abs=1e-12)


##########################
### Client initial offset ###
##########################


def test_client_start_is_offset_radially_outward_by_the_configured_distance():
    c = cfg(client_initial_offset_m=0.5)
    o = oc.OrbitReference.from_cfg(c, (0, 0, 0))
    for th in (0.0, 0.7, 2.5):
        s = oc.client_start_point(c, o, th)
        ok, dist, _ = oc.initial_offset_check(c, o, s)
        assert ok and dist == pytest.approx(0.5, abs=1e-9)
        assert np.linalg.norm(s - o.center) == pytest.approx(4.5)


def test_client_start_offset_radially_inward_and_along_a_fixed_direction():
    c = cfg(client_initial_offset_m=0.5, client_offset_direction="radial_inward")
    o = oc.OrbitReference.from_cfg(c, (0, 0, 0))
    s = oc.client_start_point(c, o, 0.0)
    assert np.linalg.norm(s - o.center) == pytest.approx(3.5)
    c = cfg(client_initial_offset_m=0.5, client_offset_direction=[0.0, 0.0, 2.0])  # normal to the plane, normalised
    o = oc.OrbitReference.from_cfg(c, (0, 0, 0))
    s = oc.client_start_point(c, o, 1.0)
    assert oc.initial_offset_check(c, o, s)[1] == pytest.approx(0.5)


def test_a_client_that_starts_on_the_orbit_fails():
    c = cfg()
    o = oc.OrbitReference.from_cfg(c, (0, 0, 0))
    ok, dist, _ = oc.initial_offset_check(c, o, o.point_at(0.4))
    assert not ok and dist == pytest.approx(0.0, abs=1e-12)
    # ... and one at a wrong distance fails too
    assert not oc.initial_offset_check(c, o, o.point_at(0.4) + 0.2 * o.outward_at(0.4))[0]


def test_offset_not_larger_than_the_arrival_tolerance_is_a_config_error():
    with pytest.raises(ValueError, match="client_initial_offset_m"):
        cfg(client_initial_offset_m=0.04, arrival_tolerance_m=0.05)


@pytest.mark.parametrize("shape", ["circle", "ellipse"])
def test_derived_centre_puts_the_client_exactly_the_offset_off_the_orbit(shape):
    c = cfg(shape=shape, client_initial_offset_m=0.5)
    start, axis, base = np.array([6.0, 1.0, 3.0]), np.array([0.0, 1.0, 0.0]), np.zeros(3)
    cands = oc.derive_center_candidates(c, start, axis, base)
    assert len(cands) == 2
    for centre, th in cands:
        o = oc.OrbitReference.from_cfg(c, centre)
        ok, dist, _ = oc.initial_offset_check(c, o, start)
        assert ok and dist == pytest.approx(0.5, abs=1e-6)
        # the nominal point is the closest one and the offset is along the outward normal
        assert np.allclose(o.point_at(th), o.closest(start)[0], atol=1e-6)
        # perpendicular to the docking axis: the axis line stays >= offset from the orbit
        far = oc.min_distance_to_segment(o.points(4096), start - 6 * axis, start + 6 * axis)
        assert far >= 0.5 - 2e-3


def test_derived_centre_prefers_the_side_pointing_away_from_the_arm_base():
    c = cfg()
    start, axis = np.array([6.0, 1.0, 3.0]), np.array([0.0, 1.0, 0.0])
    (c0, th0), _ = oc.derive_center_candidates(c, start, axis, np.zeros(3))
    o = oc.OrbitReference.from_cfg(c, c0)
    assert oc.offset_direction(c, o, th0) @ start > 0.0


def test_explicit_nominal_angle_gives_a_single_candidate():
    c = cfg(nominal_angle_deg=30.0)
    cands = oc.derive_center_candidates(c, np.array([6.0, 1.0, 3.0]), np.array([0.0, 1.0, 0.0]), np.zeros(3))
    assert len(cands) == 1 and cands[0][1] == pytest.approx(math.radians(30.0))


#####################
### Sensor keep-out ###
#####################


def test_min_distance_to_segment():
    pts = np.array([[0.0, 1.0, 0.0], [5.0, 0.0, 0.0]])
    assert oc.min_distance_to_segment(pts, [-1, 0, 0], [1, 0, 0]) == pytest.approx(1.0)
    assert oc.min_distance_to_segment(pts, [0, 0, 0], [1, 0, 0]) == pytest.approx(1.0)  # clamped to the end point


def test_sensor_keepout_flags_an_orbit_crossing_the_docking_axis():
    o = circle(center=(0, 0, 0))
    crossing = {"docking_axis": (np.array([4.0, -3.0, 0.0]), np.array([4.0, 3.0, 0.0]))}
    clear = {"docking_axis": (np.array([9.0, -3.0, 0.0]), np.array([9.0, 3.0, 0.0]))}
    ok, d = oc.sensor_keepout_check(o, crossing, 0.4)
    assert not ok and d["docking_axis"] < 0.01
    ok, d = oc.sensor_keepout_check(o, clear, 0.4)
    assert ok and d["docking_axis"] == pytest.approx(5.0, abs=0.01)


###########################
### Target selection ###
###########################


def test_target_is_the_nearest_orbit_point_first():
    o = circle(center=(0, 0, 0))
    client = np.array([4.5, 0.0, 0.0])
    cands = oc.target_candidates(o, client)
    assert cands[0].distance_m == pytest.approx(0.5)
    assert np.allclose(cands[0].point, [4.0, 0.0, 0.0])
    d = [c.distance_m for c in cands[1:]]
    assert d == sorted(d)


def test_select_target_takes_the_nearest_feasible_point():
    o = circle(center=(0, 0, 0))
    client = np.array([4.5, 0.0, 0.0])
    everything = lambda p: (True, "ok")
    t, tried = oc.select_target(o, client, everything)
    assert t.distance_m == pytest.approx(0.5) and len(tried) == 1

    # forbid the region around the nearest point (x > 3.9): the next feasible point is picked, still close
    no_x = lambda p: (bool(p[0] < 3.9), "x limit")
    t, tried = oc.select_target(o, client, no_x)
    assert t.point[0] < 3.9 and len(tried) > 1 and not tried[0]["feasible"]
    assert t.distance_m > 0.5

    # nothing feasible -> None and a log of what was tried
    t, tried = oc.select_target(o, client, lambda p: (False, "no"), max_candidates=5)
    assert t is None and len(tried) == 5


def test_reach_feasible_window():
    base = np.zeros(3)
    assert oc.reach_feasible([5, 0, 0], base, 1.0, 8.0)[0]
    assert not oc.reach_feasible([9, 0, 0], base, 1.0, 8.0)[0]
    assert not oc.reach_feasible([0.5, 0, 0], base, 1.0, 8.0)[0]


def test_joint_margin():
    assert oc.joint_margin(np.array([0.0, 1.0]), np.array([-1.0, -1.0]), np.array([2.0, 1.1])) == pytest.approx(0.1)
    assert oc.joint_margin(np.array([2.5]), np.array([-1.0]), np.array([2.0])) < 0.0


#######################
### Judgement ###
#######################


def test_free_flight_ok_and_deviations():
    drift = oc.DriftCfg()
    v = np.array([0.0, -0.01, 0.0])
    assert oc.free_flight_ok(drift, v, v, np.zeros(3))[0]
    assert not oc.free_flight_ok(drift, v * 0.5, v, np.zeros(3))[0]          # half the speed: damping / a push
    assert oc.free_flight_ok(drift, v * 0.95, v, np.zeros(3))[0]             # 0.5 mm/s: inside the tolerance
    ok, why = oc.free_flight_ok(drift, v, v, np.array([0.0, 0.0, 0.01]))      # spinning
    assert not ok and "angular" in why


def test_arrival_ok_needs_every_condition():
    c = cfg(arrival_tolerance_m=0.05)
    assert oc.arrival_ok(c, 0.03, True, 0.1, 0.01, 0.0001)[0]
    for kw in (dict(orbit_error_m=0.06), dict(joint_valid=False), dict(drift_mm=6.0), dict(drift_deg=1.0),
               dict(client_w_rad_s=0.02), dict(physics_ok=False)):
        args = dict(orbit_error_m=0.03, joint_valid=True, drift_mm=0.1, drift_deg=0.01, client_w_rad_s=0.0001)
        args.update(kw)
        ok, bad = oc.arrival_ok(c, **args)
        assert not ok and len(bad) == 1


#####################
### Configuration ###
#####################


def test_default_config_leaves_the_existing_scenarios_untouched():
    v = load_vision_config()
    assert v.orbit_reference.enabled is False
    assert v.docking.satellite_velocity_mps == 0.0


def test_orbit_switch_applies_the_drift_section_and_forbids_rotation():
    v = load_vision_config(None, ["orbit_reference.enabled=true"])
    assert np.allclose(v.mep.linear_velocity_w(), [0.0, -0.01, 0.0])
    sat = np.array(v.docking.satellite_drift_direction) * v.docking.satellite_velocity_mps
    assert np.allclose(sat, [0.0, -0.01, 0.0])
    assert np.allclose(v.mep.angular_velocity_w(), 0.0)
    v = load_vision_config(None, ["orbit_reference.enabled=true", "drift.client_velocity_mps=0.0"])
    assert v.docking.satellite_velocity_mps == 0.0

    with pytest.raises(ValueError, match="rotation"):
        load_vision_config(None, ["orbit_reference.enabled=true", "drift.angular_velocity_rad_s=[0.0, 0.0, 0.01]"])
    with pytest.raises(ValueError, match="translation_only"):
        load_vision_config(None, ["orbit_reference.enabled=true", "mep.motion_mode=six_dof"])


def test_config_validation():
    for bad in (dict(shape="spiral"), dict(radius_m=0.0), dict(normal_w=[0, 0, 0]), dict(color_rgb=[2, 0, 0]),
                dict(client_offset_direction="sideways"), dict(client_reference="tail"), dict(prim_path="/World/envs/env_0/x"),
                dict(shape="ellipse", semi_major_m=2.0, semi_minor_m=3.0), dict(hold_duration_s=0.0),
                dict(arm_reach_min_m=9.0)):
        with pytest.raises(ValueError):
            cfg(**bad)
    with pytest.raises(ValueError):
        oc.validate_drift_cfg(oc.DriftCfg(client_velocity_mps=0.01, client_direction_w=[0, 0, 0]))
    with pytest.raises(ValueError):
        load_vision_config(None, ["orbit_reference.enabled=true", "orbit_reference.center_w=[1, 2]"])


###############
### Visual ###
###############


def test_tube_mesh_is_closed_and_consistent():
    o = circle(center=(0, 0, 0))
    pts = o.points(128)
    verts, normals, counts, idx = oc.tube_mesh(pts, o.normal, 0.03, sides=8)
    assert verts.shape == (128 * 8, 3) and normals.shape == verts.shape
    assert len(counts) == 128 * 8 and len(idx) == 4 * len(counts) and max(idx) == len(verts) - 1
    # every vertex is `radius` from its centre-line point, every normal is a unit vector
    off = verts.reshape(128, 8, 3) - pts[:, None, :]
    assert np.allclose(np.linalg.norm(off, axis=-1), 0.03)
    assert np.allclose(np.linalg.norm(normals, axis=1), 1.0)


################
### Arc (Earth) ###
################


def shipped():
    """The orbit exactly as the shipped config builds it, with the Client at a made-up start point."""
    v = load_vision_config(None, ["orbit_reference.enabled=true"])
    o = v.orbit_reference
    s0, axis = np.array([15.24, 0.29, 3.0]), np.array([1.0, 0.0, 0.0])
    (centre, th), = oc.derive_center_candidates(o, s0, axis, np.zeros(3))
    return v, o, oc.OrbitReference.from_cfg(o, centre, th), s0, th


def test_default_orbit_is_a_horizontal_ring_concentric_with_the_earth_axis():
    v, o, orb, s0, th = shipped()
    assert np.allclose(orb.normal, [0, 0, 1])                # ring axis = -nadir of the sky dome
    assert th == pytest.approx(0.0)                          # nominal point: +X of the centre
    assert np.allclose(orb.center, s0 - [50.0, 0.0, 0.5])   # 50 m towards the arm side, in the plane 0.5 m below the Client
    assert np.allclose(orb.point_at(th), s0 - [0, 0, 0.5])   # the ring passes 0.5 m below the Client
    assert oc.client_start_point(o, orb, th) == pytest.approx(s0)
    ok, dist, _ = oc.initial_offset_check(o, orb, s0)
    assert ok and dist == pytest.approx(0.5, abs=1e-9)
    assert oc.offset_direction(o, orb, th) == pytest.approx([0, 0, 1])   # straight up: an altitude error


def test_only_the_visible_arc_is_used():
    v, o, orb, s0, th = shipped()
    assert not orb.is_full and math.degrees(orb.arc_span) == pytest.approx(80.0)
    lo, hi = orb.theta_range
    assert (lo, hi) == pytest.approx((-math.radians(40), math.radians(40)))
    pts = orb.points(128)
    assert len(pts) == 128
    assert np.allclose(pts[0], orb.point_at(lo)) and np.allclose(pts[-1], orb.point_at(hi))   # both end points
    assert np.allclose(np.linalg.norm(pts - orb.center, axis=1), 50.0)
    assert orb.in_arc(0.0) and orb.in_arc(math.radians(39)) and not orb.in_arc(math.radians(41)) and not orb.in_arc(math.pi)
    # arc length = radius * span (the far side of the Earth-concentric ring is not part of the orbit)
    assert oc.min_distance_to_segment(pts, orb.center, orb.center) == pytest.approx(50.0)


def test_closest_point_is_clamped_to_the_arc():
    v, o, orb, s0, th = shipped()
    q, t, d = orb.closest(orb.point_at(math.radians(20)) + [0, 0, 0.3])
    assert t == pytest.approx(math.radians(20)) and d == pytest.approx(0.3)
    # a point straight above the ring at 90 deg (outside the arc): the nearest END of the arc
    far = orb.point_at(math.radians(90))
    q, t, d = orb.closest(far)
    assert t == pytest.approx(math.radians(40)) and np.allclose(q, orb.point_at(math.radians(40)))
    q, t, d = orb.closest(orb.point_at(math.radians(-100)))
    assert t == pytest.approx(-math.radians(40))


def test_target_candidates_stay_on_the_arc():
    v, o, orb, s0, th = shipped()
    cands = oc.target_candidates(orb, s0)
    assert cands[0].point == pytest.approx(s0 - [0, 0, 0.5]) and cands[0].distance_m == pytest.approx(0.5)
    assert all(orb.in_arc(c.theta) for c in cands)
    t, tried = oc.select_target(orb, s0, lambda p: (bool(p[1] > s0[1] + 1.0), "y limit"))   # forbid the nominal region
    assert t is not None and t.point[1] > s0[1] + 1.0 and orb.in_arc(t.theta)


def test_drift_is_along_the_orbit_tangent():
    v, o, orb, s0, th = shipped()
    tangent = orb.tangent_at(th)
    assert np.allclose(np.abs(tangent), [0, 1, 0], atol=1e-9)        # world Y, horizontal, perpendicular to the +X approach axis
    for vel in (v.mep.linear_velocity_w(), np.array(v.docking.satellite_drift_direction) * v.docking.satellite_velocity_mps):
        assert abs(vel @ tangent) / np.linalg.norm(vel) == pytest.approx(1.0)
        assert vel[0] == 0.0                                          # nothing along the arm's approach axis
    # 1.5 m of drift along the tangent barely leaves the (curved) arc: the Client keeps its offset
    moved = s0 + 1.5 * np.array([0, -1, 0])
    assert orb.distance(moved) == pytest.approx(0.5, abs=0.03)


def test_the_shipped_orbit_keeps_the_docking_axis_and_the_wrist_corridor_clear():
    v, o, orb, s0, th = shipped()
    axis_seg = (s0 - 6 * np.array([1.0, 0, 0]), s0 + 3 * np.array([1.0, 0, 0]))
    corridor = (np.array([5.0, -2.75, 3.0]), np.array([5.0, -2.75, 3.0]) + np.array([1.0, 0, 0]) * 1.6)
    ok, dist = oc.sensor_keepout_check(orb, {"docking_axis": axis_seg, "wrist": corridor}, o.sensor_keepout_m)
    assert ok and dist["docking_axis"] >= 0.5 - 1e-6


def test_view_puts_the_arc_parallel_to_the_limb_above_it():
    """From the ring axis, at the configured height, the arc is a circle of constant elevation
    -view_elevation, above the Earth limb (-18.3 deg)."""
    v, o, orb, s0, th = shipped()
    eye = orb.center + orb.normal * (orb.a * math.tan(math.radians(o.view_elevation_deg)))
    pts = orb.points(64)
    d = pts - eye
    elev = np.degrees(np.arctan2(d @ orb.normal, np.linalg.norm(d - np.outer(d @ orb.normal, orb.normal), axis=1)))
    assert np.allclose(elev, -o.view_elevation_deg, atol=1e-6)
    assert -oc.EARTH_LIMB_DIP_DEG < elev.max() < 0.0


def test_open_arc_tube_mesh():
    v, o, orb, s0, th = shipped()
    pts = orb.points(128)
    verts, normals, counts, idx = oc.tube_mesh(pts, orb.normal, 0.03, sides=8, closed=False)
    assert verts.shape == (128 * 8, 3)
    assert len(counts) == 127 * 8 and max(idx) == len(verts) - 1          # no segment wraps from the last point to the first
    assert np.allclose(np.linalg.norm(verts.reshape(128, 8, 3) - pts[:, None, :], axis=-1), 0.03)


def test_arc_config_validation():
    for bad in (dict(arc_span_deg=2.0), dict(arc_span_deg=400.0), dict(view_elevation_deg=0.0), dict(view_elevation_deg=90.0)):
        with pytest.raises(ValueError):
            cfg(**bad)
