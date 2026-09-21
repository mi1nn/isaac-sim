"""Client-satellite reference orbit and the orbit-return phase: geometry, config, judgement.

Pure numpy (no Isaac Sim imports), so every rule here is unit-testable offline
(`project/tests/test_orbit_return.py`). The state machine that uses it lives in
`vision_capture_demo.py`; the red orbit prim is authored in `vision_task.py`.

The scene is zero gravity, so the "normal orbit" is NOT a Keplerian solution: it is a
**reference trajectory in the world frame** (an arc of a circle or an ellipse in a plane) that
the Client satellite has to be brought back onto after it has left it. The Earth of the scene is
the sky-dome image (`low_earth_orbit.exr`): a cap around the world -Z axis (nadir) whose limb sits
`EARTH_LIMB_DIP_DEG` below the horizon, so the default orbit is a horizontal ring concentric with
that cap (normal = +Z) of which only the visible arc is drawn. See `docs/mrv_orbit_return.md`.

Frames / conventions (as in `frames.py`, `probe_dock.py`): world frame W, lengths [m],
angles [deg] in config and [rad] internally, angular velocities [rad/s].

    orbit plane   : origin `center`, normal `normal`, in-plane axes e1, e2 (e1 x e2 = normal)
    orbit point   : P(theta) = center + a cos(theta) e1 + b sin(theta) e2
                    circle: a = b = radius; ellipse: a = semi_major, b = semi_minor
    Client point  : the rigid point of the Client used for "on the orbit" (`client_reference`)
"""

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

##############
### Config ###
##############

SHAPES = ("circle", "ellipse")
TARGET_MODES = ("nearest_feasible_point",)
# The rigid Client point that has to reach the orbit:
#   sat_dock_point : SAT_DOCK_POINT (the docking frame; coincides with the probe tip once docked)
#   body_origin    : the satellite body origin
CLIENT_REFERENCES = ("sat_dock_point", "body_origin")
RADIAL_DIRECTIONS = ("radial_outward", "radial_inward")
# Measured from `assets/srb_assets/skydome/low_res/low_earth_orbit.exr` (Domain.ORBIT sky dome): the
# Earth limb is a circle of constant elevation, its row at 60.2 % of the 2:1 equirectangular image
# -> 18.3 deg below the horizon, i.e. the Earth is a cap of 71.7 deg angular radius around nadir (-Z)
EARTH_LIMB_DIP_DEG = 18.3


@dataclass
class OrbitReferenceCfg:
    """`orbit_reference:` section of `config/vision_capture.yaml` (units in the YAML)."""

    # Environment on (red orbit, drifting Client, checks). `vision_capture.py --orbit-return`
    # / `--orbit-only` switch it on; the default leaves the existing scenarios untouched.
    enabled: bool = False
    # true (`--orbit-only`): only build the scene and verify it (orbit prim, free flight,
    # initial offset) for `observe_only_duration_s`, no capture / docking / return
    observe_only: bool = False
    observe_only_duration_s: float = 20.0

    ## Orbit geometry (world frame)
    shape: str = "circle"
    # null: derived so that the Client's existing (validated) start point lies exactly
    # `client_initial_offset_m` off the orbit. A list moves the Client to nominal point + offset.
    center_w: Optional[List[float]] = None
    normal_w: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    radius_m: float = 50.0
    semi_major_m: float = 50.0
    semi_minor_m: float = 40.0
    # Only the part of the ring in front of the viewer is drawn / used: an arc of this angular length
    # (deg, about the axis through the centre), centred on `nominal_angle_deg`. 360 = the whole ring.
    arc_span_deg: float = 80.0
    # GUI start view: the eye sits on the ring axis, this far above the ring plane in angle, i.e. the
    # ring appears `view_elevation_deg` below the horizon (the Earth limb is EARTH_LIMB_DIP_DEG below it)
    view_elevation_deg: float = 12.0
    segments: int = 128
    # Visual only (a thin emissive tube along the orbit: BasisCurves are not reliably drawn by RTX)
    line_width_m: float = 0.25
    color_rgb: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    prim_path: str = "/World/reference_orbit"

    ## Client start
    # Angle (about `normal_w`, from the in-plane axis e1) of the orbit point the Client was
    # nominally on = the centre of the drawn arc. null: perpendicular to the docking axis (center
    # null) or the point nearest to the Client's start (center given).
    nominal_angle_deg: Optional[float] = 0.0
    client_reference: str = "sat_dock_point"
    client_initial_offset_m: float = 0.5
    # "radial_outward" | "radial_inward" | [x, y, z] (world direction). Default: straight up (an altitude
    # error above the ring plane): the docking axis then stays `client_initial_offset_m` from the orbit.
    client_offset_direction: Union[str, List[float]] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    initial_offset_tolerance_m: float = 0.005

    ## Return
    target_mode: str = "nearest_feasible_point"
    arrival_tolerance_m: float = 0.05
    hold_duration_s: float = 10.0
    # Arm workspace pre-filter for the EE contact point (which holds the MEP), from the arm base [m]
    arm_reach_min_m: float = 1.0
    arm_reach_max_m: float = 8.0
    max_target_candidates: int = 64
    # Transfer speed (the acceleration / deceleration profile is `docking.accel_mps2` /
    # `docking.decel_gain_hz`, the same as the existing docking motions)
    transfer_speed_mps: float = 0.02
    # MEP + Client swing as a pendulum-like mode of ~45 s period (measured), so the profile has to be slow and
    # smooth: a gentle ramp and an exponential approach with a ~30 s time constant instead of the docking values
    transfer_accel_mps2: float = 0.002
    transfer_decel_gain_hz: float = 0.03
    # The reference speed fades to 0 as the measured tool lag reaches this [m] (a heavy MEP + Client follows the
    # reference slowly; a reference that runs ahead saturates the joint step and the arm stops short of the goal)
    transfer_lag_limit_m: float = 0.05
    transfer_timeout_s: float = 400.0
    arrival_hold_s: float = 0.5
    arrival_timeout_s: float = 60.0
    max_retargets: int = 3
    # Stop (safe stop) when a joint gets closer than this to its limit [rad]
    joint_limit_margin_rad: float = 0.02
    # MEP-Client relative pose drift while returning / holding (the existing DOCK6 hold criteria)
    max_relative_drift_mm: float = 5.0
    max_relative_drift_deg: float = 0.5
    max_client_angular_velocity_rad_s: float = 0.005
    # The orbit must stay this far from the docking axis and the wrist-camera-to-tags corridor
    # (a visible line in front of a sensor would corrupt the AprilTag pose / the depth reading)
    sensor_keepout_m: float = 0.4
    # No jump of the Client point larger than this in one control step: nothing is teleported [m]
    max_step_jump_m: float = 0.005


@dataclass
class DriftCfg:
    """`drift:` section: free drift of the MEP and the Client. Applied (over `mep.*` and
    `docking.satellite_*`) only while `orbit_reference.enabled`."""

    mep_velocity_mps: float = 0.01
    mep_direction_w: List[float] = field(default_factory=lambda: [0.0, -1.0, 0.0])
    client_velocity_mps: float = 0.01
    client_direction_w: List[float] = field(default_factory=lambda: [0.0, -1.0, 0.0])
    # Must stay zero in this scenario (no rotation of either body)
    angular_velocity_rad_s: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Free flight monitor: a free (not yet captured / docked) body whose velocity differs from the
    # commanded one by more than this, or that turns faster than the angular limit, fails the run
    velocity_tolerance_mps: float = 0.002
    angular_tolerance_rad_s: float = 0.002

    def mep_velocity_w(self) -> np.ndarray:
        return _unit(self.mep_direction_w) * float(self.mep_velocity_mps)

    def client_velocity_w(self) -> np.ndarray:
        return _unit(self.client_direction_w) * float(self.client_velocity_mps)


def _vec3(x, name: str) -> np.ndarray:
    v = np.asarray(x, dtype=float)
    if v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError(f"{name} must be 3 finite values, got {x}")
    return v


def _unit(x) -> np.ndarray:
    v = np.asarray(x, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros(3)


def validate_orbit_cfg(cfg: OrbitReferenceCfg):
    """Raise on a configuration that cannot work (called from `load_vision_config`)."""
    if cfg.shape not in SHAPES:
        raise ValueError(f"orbit_reference.shape must be one of {SHAPES}, got '{cfg.shape}'")
    if cfg.center_w is not None:
        cfg.center_w = _vec3(cfg.center_w, "orbit_reference.center_w").tolist()
    n = _vec3(cfg.normal_w, "orbit_reference.normal_w")
    if np.linalg.norm(n) < 1e-9:
        raise ValueError("orbit_reference.normal_w must be non-zero")
    cfg.normal_w = (n / np.linalg.norm(n)).tolist()
    axes = (cfg.radius_m,) if cfg.shape == "circle" else (cfg.semi_major_m, cfg.semi_minor_m)
    if any(not math.isfinite(x) or x <= 0.0 for x in axes):
        raise ValueError("orbit_reference radius_m (circle) / semi_major_m and semi_minor_m (ellipse) must be > 0")
    if cfg.shape == "ellipse" and cfg.semi_minor_m > cfg.semi_major_m:
        raise ValueError("orbit_reference.semi_minor_m must be <= semi_major_m")
    if not 5.0 <= cfg.arc_span_deg <= 360.0:
        raise ValueError("orbit_reference.arc_span_deg must be in [5, 360]")
    if not 0.0 < cfg.view_elevation_deg < 89.0:
        raise ValueError("orbit_reference.view_elevation_deg must be in (0, 89)")
    if cfg.segments < 128:
        raise ValueError("orbit_reference.segments must be >= 128 (a visibly smooth line)")
    if cfg.line_width_m <= 0.0:
        raise ValueError("orbit_reference.line_width_m must be > 0")
    c = _vec3(cfg.color_rgb, "orbit_reference.color_rgb")
    if not ((c >= 0.0) & (c <= 1.0)).all():
        raise ValueError("orbit_reference.color_rgb must be in [0, 1]")
    if not cfg.prim_path.startswith("/") or cfg.prim_path.startswith("/World/envs"):
        raise ValueError("orbit_reference.prim_path must be an absolute path outside /World/envs")
    if cfg.client_reference not in CLIENT_REFERENCES:
        raise ValueError(f"orbit_reference.client_reference must be one of {CLIENT_REFERENCES}, got '{cfg.client_reference}'")
    if cfg.target_mode not in TARGET_MODES:
        raise ValueError(f"orbit_reference.target_mode must be one of {TARGET_MODES}, got '{cfg.target_mode}'")
    if isinstance(cfg.client_offset_direction, str):
        if cfg.client_offset_direction not in RADIAL_DIRECTIONS:
            raise ValueError(f"orbit_reference.client_offset_direction must be one of {RADIAL_DIRECTIONS} or a 3-vector, "
                             f"got '{cfg.client_offset_direction}'")
    else:
        d = _vec3(cfg.client_offset_direction, "orbit_reference.client_offset_direction")
        if np.linalg.norm(d) < 1e-9:
            raise ValueError("orbit_reference.client_offset_direction must be non-zero")
        cfg.client_offset_direction = (d / np.linalg.norm(d)).tolist()
    # The Client must start OFF the orbit, by more than the arrival tolerance (else there is nothing to return)
    if cfg.client_initial_offset_m <= cfg.arrival_tolerance_m:
        raise ValueError("orbit_reference.client_initial_offset_m must be larger than arrival_tolerance_m "
                         "(a Client that starts on the orbit has nothing to return from)")
    if cfg.arrival_tolerance_m <= 0.0 or cfg.initial_offset_tolerance_m <= 0.0:
        raise ValueError("orbit_reference.arrival_tolerance_m and initial_offset_tolerance_m must be > 0")
    if cfg.hold_duration_s <= 0.0 or cfg.observe_only_duration_s <= 0.0:
        raise ValueError("orbit_reference.hold_duration_s and observe_only_duration_s must be > 0")
    if not 0.0 <= cfg.arm_reach_min_m < cfg.arm_reach_max_m:
        raise ValueError("orbit_reference requires 0 <= arm_reach_min_m < arm_reach_max_m")
    if cfg.transfer_speed_mps <= 0.0 or cfg.max_target_candidates < 1 or cfg.max_retargets < 0:
        raise ValueError("orbit_reference.transfer_speed_mps > 0, max_target_candidates >= 1, max_retargets >= 0 required")
    if cfg.transfer_accel_mps2 <= 0.0 or cfg.transfer_decel_gain_hz <= 0.0:
        raise ValueError("orbit_reference.transfer_accel_mps2 and transfer_decel_gain_hz must be > 0")
    if cfg.transfer_lag_limit_m <= 0.0:
        raise ValueError("orbit_reference.transfer_lag_limit_m must be > 0")
    if cfg.sensor_keepout_m < 0.0 or cfg.max_step_jump_m <= 0.0:
        raise ValueError("orbit_reference.sensor_keepout_m >= 0 and max_step_jump_m > 0 required")


def validate_drift_cfg(cfg: DriftCfg):
    """No rotation, sane velocities (only meaningful while the orbit scenario is on)."""
    w = _vec3(cfg.angular_velocity_rad_s, "drift.angular_velocity_rad_s")
    if float(np.linalg.norm(w)) != 0.0:
        raise ValueError("drift.angular_velocity_rad_s must be [0, 0, 0]: the orbit-return scenario forbids rotation "
                         "of the MEP and the Client")
    cfg.angular_velocity_rad_s = [0.0, 0.0, 0.0]
    for who in ("mep", "client"):
        v, d = float(getattr(cfg, f"{who}_velocity_mps")), _vec3(getattr(cfg, f"{who}_direction_w"), f"drift.{who}_direction_w")
        if v < 0.0 or not math.isfinite(v):
            raise ValueError(f"drift.{who}_velocity_mps must be >= 0")
        if v > 0.0 and np.linalg.norm(d) < 1e-9:
            raise ValueError(f"drift.{who}_direction_w must be non-zero when {who}_velocity_mps > 0")
        setattr(cfg, f"{who}_direction_w", _unit(d).tolist())
    if cfg.velocity_tolerance_mps <= 0.0 or cfg.angular_tolerance_rad_s <= 0.0:
        raise ValueError("drift.velocity_tolerance_mps and angular_tolerance_rad_s must be > 0")


############
### Orbit ###
############


def plane_basis(normal) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(e1, e2, n) of an orbit plane: e1 = world +X projected onto the plane (world +Y if the
    normal is nearly along X), e2 = n x e1, so e1 x e2 = n."""
    n = _unit(normal)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = _unit(ref - float(ref @ n) * n)
    return e1, np.cross(n, e1), n


@dataclass
class OrbitReference:
    """Circle / ellipse `P(theta) = center + a cos(theta) e1 + b sin(theta) e2`."""

    center: np.ndarray
    normal: np.ndarray
    a: float
    b: float
    e1: np.ndarray
    e2: np.ndarray
    # The usable / drawn part of the curve: theta in [arc_centre - span/2, arc_centre + span/2]
    arc_centre: float = 0.0
    arc_span: float = 2.0 * math.pi

    @staticmethod
    def from_cfg(cfg: OrbitReferenceCfg, center, arc_centre_rad: Optional[float] = None) -> "OrbitReference":
        e1, e2, n = plane_basis(cfg.normal_w)
        a, b = (cfg.radius_m, cfg.radius_m) if cfg.shape == "circle" else (cfg.semi_major_m, cfg.semi_minor_m)
        if arc_centre_rad is None:
            arc_centre_rad = math.radians(cfg.nominal_angle_deg) if cfg.nominal_angle_deg is not None else 0.0
        return OrbitReference(np.asarray(center, dtype=float), n, float(a), float(b), e1, e2,
                              float(arc_centre_rad), math.radians(min(cfg.arc_span_deg, 360.0)))

    @property
    def is_full(self) -> bool:
        return self.arc_span >= 2.0 * math.pi - 1e-9

    @property
    def theta_range(self) -> Tuple[float, float]:
        return self.arc_centre - 0.5 * self.arc_span, self.arc_centre + 0.5 * self.arc_span

    def in_arc(self, theta: float) -> bool:
        if self.is_full:
            return True
        d = (theta - self.arc_centre + math.pi) % (2.0 * math.pi) - math.pi
        return abs(d) <= 0.5 * self.arc_span + 1e-12

    @property
    def is_circle(self) -> bool:
        return abs(self.a - self.b) < 1e-12

    def point_at(self, theta) -> np.ndarray:
        """Orbit point(s): scalar theta -> (3,), array -> (N, 3)."""
        t = np.asarray(theta, dtype=float)
        return self.center + np.multiply.outer(self.a * np.cos(t), self.e1) + np.multiply.outer(self.b * np.sin(t), self.e2)

    def thetas(self, n: int) -> np.ndarray:
        """`n` angles over the arc (both end points included) or, for a full ring, around it."""
        lo, hi = self.theta_range
        return np.linspace(lo, hi, int(n), endpoint=not self.is_full)

    def points(self, n: int) -> np.ndarray:
        return self.point_at(self.thetas(n))

    def outward_at(self, theta: float) -> np.ndarray:
        """In-plane unit normal of the curve pointing away from the centre (radial for a circle)."""
        v = math.cos(theta) / self.a * self.e1 + math.sin(theta) / self.b * self.e2
        return _unit(v)

    def closest(self, p) -> Tuple[np.ndarray, float, float]:
        """(closest point of the arc, its theta, distance) to the world point `p` (3D). Beyond the
        arc ends the nearest end point is returned."""
        pw = np.asarray(p, dtype=float)
        d = pw - self.center
        x, y = float(d @ self.e1), float(d @ self.e2)
        if self.is_circle:
            theta = math.atan2(y, x) if math.hypot(x, y) > 1e-12 else self.arc_centre
        else:
            theta = self._closest_theta(x, y)
        if not self.in_arc(theta):
            ends = [self.theta_range[0], self.theta_range[1]]
            theta = min(ends, key=lambda t: float(np.linalg.norm(pw - self.point_at(t))))
        q = self.point_at(theta)
        return q, theta, float(np.linalg.norm(pw - q))

    def _closest_theta(self, x: float, y: float) -> float:
        n = 1440
        th = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        i = int(np.argmin((x - self.a * np.cos(th)) ** 2 + (y - self.b * np.sin(th)) ** 2))
        step = 2.0 * math.pi / n

        def f(t):
            return (x - self.a * math.cos(t)) ** 2 + (y - self.b * math.sin(t)) ** 2

        lo, hi = th[i] - step, th[i] + step
        g = (math.sqrt(5.0) - 1.0) / 2.0
        for _ in range(80):  # golden-section on the bracket around the coarse minimum
            c, e = hi - g * (hi - lo), lo + g * (hi - lo)
            if f(c) < f(e):
                hi = e
            else:
                lo = c
        return (0.5 * (lo + hi)) % (2.0 * math.pi)

    def distance(self, p) -> float:
        return self.closest(p)[2]

    def out_of_plane(self, p) -> float:
        return float((np.asarray(p, dtype=float) - self.center) @ self.normal)

    def describe(self) -> Dict[str, object]:
        return {"center_w": self.center.tolist(), "normal_w": self.normal.tolist(), "e1_w": self.e1.tolist(),
                "e2_w": self.e2.tolist(), "semi_axis_a_m": self.a, "semi_axis_b_m": self.b,
                "shape": "circle" if self.is_circle else "ellipse",
                "arc_centre_deg": math.degrees(self.arc_centre), "arc_span_deg": math.degrees(self.arc_span),
                "arc_end_points_w": [self.point_at(t).tolist() for t in self.theta_range]}

    def tangent_at(self, theta: float) -> np.ndarray:
        """Unit tangent for increasing theta."""
        return _unit(-self.a * math.sin(theta) * self.e1 + self.b * math.cos(theta) * self.e2)


def offset_direction(cfg: OrbitReferenceCfg, orbit: OrbitReference, theta: float) -> np.ndarray:
    """Unit world direction of the Client's initial offset from the orbit point at `theta`."""
    d = cfg.client_offset_direction
    if isinstance(d, str):
        out = orbit.outward_at(theta)
        return out if d == "radial_outward" else -out
    return _unit(d)


def client_start_point(cfg: OrbitReferenceCfg, orbit: OrbitReference, theta: float) -> np.ndarray:
    """Where the Client point starts: the nominal orbit point + offset * direction."""
    return orbit.point_at(theta) + float(cfg.client_initial_offset_m) * offset_direction(cfg, orbit, theta)


def derive_center_candidates(cfg: OrbitReferenceCfg, client_start, dock_axis_w, base_w) -> List[Tuple[np.ndarray, float]]:
    """Orbit centres (and the nominal theta) for which `client_start` lies exactly
    `client_initial_offset_m` off the orbit, in the order they should be tried.

    The centre follows from `client_start = P(theta) + offset * dir(theta)`:
    `center = client_start - offset * dir - (a cos e1 + b sin e2)`; `dir` and the in-plane part
    do not depend on the centre. Without `nominal_angle_deg` the radial direction is chosen
    perpendicular to both the orbit normal and the docking axis, so the docking axis line stays
    at least `client_initial_offset_m` from the orbit; both signs are returned, the one whose
    offset direction points away from the arm base first (the orbit then reaches towards the arm,
    so the nearest orbit point is closer to the base than the Client).
    """
    e1, e2, n = plane_basis(cfg.normal_w)
    a, b = (cfg.radius_m, cfg.radius_m) if cfg.shape == "circle" else (cfg.semi_major_m, cfg.semi_minor_m)
    template = OrbitReference(np.zeros(3), n, float(a), float(b), e1, e2)
    s = np.asarray(client_start, dtype=float)
    if cfg.nominal_angle_deg is not None:
        thetas = [math.radians(cfg.nominal_angle_deg)]
    else:
        u = np.cross(n, _unit(dock_axis_w))
        if np.linalg.norm(u) < 1e-6:
            u = e1
        u = _unit(u)
        theta = math.atan2(b * float(u @ e2), a * float(u @ e1))  # outward normal ~ (cos/a, sin/b)
        thetas = [theta, theta + math.pi]
        away = np.asarray(s, dtype=float) - np.asarray(base_w, dtype=float)
        thetas.sort(key=lambda t: -float(offset_direction(cfg, template, t) @ away))
    out = []
    for th in thetas:
        th = th % (2.0 * math.pi)
        center = s - float(cfg.client_initial_offset_m) * offset_direction(cfg, template, th) - template.point_at(th)
        out.append((center, th))
    return out


def initial_offset_check(cfg: OrbitReferenceCfg, orbit: OrbitReference, client_point) -> Tuple[bool, float, str]:
    """The Client must start `client_initial_offset_m` (+- tolerance) off the orbit."""
    dist = orbit.distance(client_point)
    err = dist - float(cfg.client_initial_offset_m)
    ok = abs(err) <= cfg.initial_offset_tolerance_m and dist > cfg.arrival_tolerance_m
    return ok, dist, (f"distance to the orbit {dist*1000:.1f} mm, configured offset {cfg.client_initial_offset_m*1000:.1f} mm "
                      f"(error {err*1000:+.2f} mm, tolerance {cfg.initial_offset_tolerance_m*1000:.1f} mm)")


######################
### Sensor keep-out ###
######################


def min_distance_to_segment(points: np.ndarray, a, b) -> float:
    """Smallest distance from any of `points` (N, 3) to the segment a-b."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    ab = b - a
    t = np.clip(((points - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(points - (a + np.outer(t, ab)), axis=1).min())


def sensor_keepout_check(orbit: OrbitReference, segments: Dict[str, Tuple[np.ndarray, np.ndarray]],
                         keepout_m: float, samples: int = 4096) -> Tuple[bool, Dict[str, float]]:
    """Distance of the orbit to each sensor ray/corridor segment; ok when all >= `keepout_m`."""
    pts = orbit.points(samples)
    dist = {name: min_distance_to_segment(pts, a, b) for name, (a, b) in segments.items()}
    return all(d >= keepout_m for d in dist.values()), dist


#############################
### Return target choice ###
#############################


@dataclass
class OrbitTarget:
    theta: float
    point: np.ndarray
    distance_m: float  # from the Client point when it was chosen


def target_candidates(orbit: OrbitReference, client_point, n_samples: int = 128) -> List[OrbitTarget]:
    """Orbit points ordered by distance from the Client point: the exact nearest point first,
    then the `n_samples` uniform samples (sorted), for the feasibility fallback."""
    c = np.asarray(client_point, dtype=float)
    q0, th0, d0 = orbit.closest(c)
    th = orbit.thetas(n_samples)
    pts = orbit.point_at(th)
    dist = np.linalg.norm(pts - c, axis=1)
    out = [OrbitTarget(th0, q0, d0)]
    out += [OrbitTarget(float(th[i]), pts[i], float(dist[i])) for i in np.argsort(dist)]
    return out


def reach_feasible(tool_goal, base_w, r_min: float, r_max: float) -> Tuple[bool, str]:
    """Arm workspace pre-filter: the EE contact point goal has to lie between `r_min` and `r_max`
    from the arm base (the joint limits are watched while moving)."""
    r = float(np.linalg.norm(np.asarray(tool_goal, dtype=float) - np.asarray(base_w, dtype=float)))
    if r > r_max:
        return False, f"{r:.2f} m from the arm base > reach {r_max:.2f} m"
    if r < r_min:
        return False, f"{r:.2f} m from the arm base < keep-out {r_min:.2f} m"
    return True, f"{r:.2f} m from the arm base"


def select_target(orbit: OrbitReference, client_point, feasible: Callable[[np.ndarray], Tuple[bool, str]],
                  max_candidates: int = 64, n_samples: int = 128) -> Tuple[Optional[OrbitTarget], List[dict]]:
    """`nearest_feasible_point`: the first candidate (nearest first) for which `feasible` holds.
    Returns (target or None, log of the candidates tried)."""
    tried = []
    for cand in target_candidates(orbit, client_point, n_samples)[: int(max_candidates)]:
        ok, why = feasible(cand.point)
        tried.append({"theta_deg": math.degrees(cand.theta), "distance_m": cand.distance_m, "feasible": bool(ok), "why": why})
        if ok:
            return cand, tried
    return None, tried


############################
### Judgement helpers ###
############################


def free_flight_deviation(v_w, v_cmd_w, w_w) -> Tuple[float, float]:
    """(|v - v_cmd| [m/s], |w| [rad/s]) of a free body."""
    return (float(np.linalg.norm(np.asarray(v_w, dtype=float) - np.asarray(v_cmd_w, dtype=float))),
            float(np.linalg.norm(np.asarray(w_w, dtype=float))))


def free_flight_ok(drift: DriftCfg, v_w, v_cmd_w, w_w) -> Tuple[bool, str]:
    dv, dw = free_flight_deviation(v_w, v_cmd_w, w_w)
    bad = []
    if dv > drift.velocity_tolerance_mps:
        bad.append(f"velocity differs from the commanded by {dv*1000:.2f} mm/s (> {drift.velocity_tolerance_mps*1000:.2f})")
    if dw > drift.angular_tolerance_rad_s:
        bad.append(f"angular velocity {dw:.5f} rad/s (> {drift.angular_tolerance_rad_s:.5f})")
    return not bad, "; ".join(bad) if bad else "ok"


def arrival_ok(cfg: OrbitReferenceCfg, orbit_error_m: float, joint_valid: bool, drift_mm: float, drift_deg: float,
               client_w_rad_s: float, physics_ok: bool = True) -> Tuple[bool, List[str]]:
    """ORBIT_ARRIVAL_CHECK: every condition must hold; returns (ok, failed conditions)."""
    bad = []
    if not orbit_error_m <= cfg.arrival_tolerance_m:
        bad.append(f"Client point {orbit_error_m*1000:.1f} mm from the orbit (> {cfg.arrival_tolerance_m*1000:.0f} mm)")
    if not joint_valid:
        bad.append("MEP-Client FixedJoint missing")
    if not (drift_mm <= cfg.max_relative_drift_mm and drift_deg <= cfg.max_relative_drift_deg):
        bad.append(f"MEP-Client relative drift {drift_mm:.3f} mm / {drift_deg:.4f} deg "
                   f"(> {cfg.max_relative_drift_mm:g} mm / {cfg.max_relative_drift_deg:g} deg)")
    if not client_w_rad_s <= cfg.max_client_angular_velocity_rad_s:
        bad.append(f"Client angular velocity {client_w_rad_s:.5f} rad/s (> {cfg.max_client_angular_velocity_rad_s:g})")
    if not physics_ok:
        bad.append("abnormal physics state (NaN / explosion / joint speed)")
    return not bad, bad


def joint_margin(q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> float:
    """Smallest distance of any joint to its limits [rad] (negative: outside)."""
    return float(np.minimum(np.asarray(q) - np.asarray(lower), np.asarray(upper) - np.asarray(q)).min())


##############
### Visual ###
##############


def tube_mesh(points: np.ndarray, normal, radius: float, sides: int = 8, closed: bool = True):
    """Tube around a *planar* polyline (the orbit ring, or an arc of it with `closed=False`):
    `(vertices, normals, face_vertex_counts, face_vertex_indices)`. Cross sections lie in the plane
    spanned by the in-plane side vector and the plane normal, so the frame is exact (no twist)."""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    nrm = _unit(normal)
    if closed:
        tangent = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    else:
        tangent = np.gradient(pts, axis=0)
    side = np.array([_unit(np.cross(t, nrm)) for t in tangent])
    phi = np.linspace(0.0, 2.0 * math.pi, int(sides), endpoint=False)
    ring = np.array([math.cos(p) * side + math.sin(p) * nrm for p in phi])  # (sides, n, 3)
    ring = np.transpose(ring, (1, 0, 2))  # (n, sides, 3): unit offsets
    verts = (pts[:, None, :] + radius * ring).reshape(-1, 3)
    normals = ring.reshape(-1, 3)
    counts, idx = [], []
    for i in range(n if closed else n - 1):
        j = (i + 1) % n
        for k in range(int(sides)):
            m = (k + 1) % int(sides)
            counts.append(4)
            idx += [i * sides + k, i * sides + m, j * sides + m, j * sides + k]
    return verts, normals, counts, idx
