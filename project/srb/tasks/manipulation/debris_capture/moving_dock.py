"""Moving-client docking: rendezvous with a drifting client, dock, stop, release, depart.

Pure numpy (no Isaac Sim imports), so every rule here is unit-testable offline
(`project/tests/test_moving_dock.py`). The state machine that uses it lives in
`vision_capture_demo.py`; the docking itself is the existing `probe_dock` phase.

Bodies (all positions / velocities in the WORLD frame, [m] and [m/s]; angular rates
[rad/s], world frame; quaternions (w, x, y, z) as everywhere in this package):

    client  the satellite (`{ENV}/satellite`, a dynamic rigid body with zero gravity)
    MEP     the captured payload (`{ENV}/debris`), held by the arm (`capture_joint`)
    MRV     the Canadarm3 base + hull (`MrvTransit`: a world-fixed articulation that
            is translated kinematically, i.e. its root pose is written every step)

Everything the rendezvous and the docking gates look at is RELATIVE to the client:

    v_rel_mrv = v_mrv - v_client          v_rel_mep = v_mep - v_client
    e_mrv     = (p_client - p_mrv) - d_ref

`d_ref` is the client-to-MRV offset of the configuration the docking pipeline was
verified in (both at their nominal poses), so holding e_mrv -> 0 and v_rel -> 0 makes
the docking see the same relative geometry as the static client did, while every body
keeps moving in the world.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

##############
### Config ###
##############


def vec3(value, name: str) -> List[float]:
    """3-vector from a list `[x, y, z]` or a mapping `{x:, y:, z:}` (YAML either way)."""
    if isinstance(value, dict):
        value = [value.get(k, 0.0) for k in ("x", "y", "z")]
    v = np.asarray(value, dtype=float).reshape(-1)
    if v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError(f"{name} must be 3 finite values [x, y, z], got {value}")
    return v.tolist()


@dataclass
class ClientMotionCfg:
    """`client:` section -- the satellite (client) free flight."""

    # Master switch of the moving-client scenario (`vision_capture.py --moving_dock`).
    # false: the satellite stays at rest and the docking runs exactly as before.
    release_enabled: bool = False
    # true: the client is released at t = 0 and drifts through the MRV approach and the
    # capture (the MRV joins it at the docking handover); false: released at the handover.
    # (--dock_only starts the docking at t = 0 either way)
    release_at_start: bool = True
    # Client drift after the release [m/s], world frame. +X is the docking (insertion)
    # axis and the arm's approach direction, so the MRV chases along its own approach
    # axis and departs straight back along it (the EE backs off the MEP face normal).
    linear_velocity_mps: List[float] = field(default_factory=lambda: [0.02, 0.0, 0.0])
    # Client angular rate [rad/s], world frame. The MRV only translates, so a rotating
    # client is NOT a verified configuration (a warning is printed when non-zero).
    angular_velocity_rad_s: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # The release is a thrust ramp (F = m a, tau = I alpha on the client), never a
    # velocity write: acceleration limits [m/s^2], [rad/s^2] and the closing time constant
    release_accel_mps2: float = 0.004
    release_ang_accel_rad_s2: float = 0.002
    release_time_constant_s: float = 0.5
    # Release complete once |v - v_cmd| and |w - w_cmd| are below these
    velocity_tolerance_mps: float = 0.0005
    angular_tolerance_rad_s: float = 0.0005
    release_timeout_s: float = 30.0
    # CLIENT_CRUISE: watch the free drift this long before the chase. 0 skips the state
    # (demo default); Test 1 of the spec uses 5-10 s
    cruise_check_s: float = 0.0
    # Free-flight check during the cruise: velocity change over the whole window
    cruise_max_velocity_change_mps: float = 0.0005
    # PHYSICS_ERROR above these (a client this fast/spinning is an explosion, not a drift)
    max_speed_mps: float = 0.2
    max_angular_rate_rad_s: float = 0.05

    def velocity(self) -> np.ndarray:
        return np.asarray(self.linear_velocity_mps, dtype=float)

    def angular_velocity(self) -> np.ndarray:
        return np.asarray(self.angular_velocity_rad_s, dtype=float)


@dataclass
class RendezvousCfg:
    """`rendezvous:` section -- MRV chase, velocity matching, rendezvous gate."""

    # MRV velocity the chase accelerates to before the closed loop takes over [m/s]
    # (world). [] skips CHASE (demo default). Test 2 of the spec sets it slower than the
    # client ([0.015, 0, 0] with `mrv_follow_during_release: false`) so the matching has
    # work to do -- which then costs a gap of ~0.2 m and ~16 s of rendezvous.
    mrv_initial_velocity_mps: List[float] = field(default_factory=list)
    # true: the MRV keeps its station on the client from the release on (no gap forms);
    # false: it waits at rest until CHASE (the spec's Test 2 setup)
    mrv_follow_during_release: bool = True
    # true: the docking transport starts at once; the client release and the MRV station
    # keeping run next to it, and the matching / station gates are checked right before
    # the docking-axis approach (no waiting states). false: the spec's sequential
    # CLIENT_RELEASE -> [CLIENT_CRUISE] -> [CHASE] -> VELOCITY_MATCHING -> RENDEZVOUS
    concurrent_docking: bool = True
    # Forced insertion (moving client only): the MEP always closes on the client along
    # the docking axis at >= `insertion_min_relative_speed_mps` once it reached the
    # pre-dock point -- no swing-settle wait, the depth reading is logged but does not
    # hold the approach, and the approach only re-aligns outside the wider corridor
    # below (or when the probe would come closer than `docking.min_wall_clearance_m` to
    # the nozzle wall). The FixedJoint still requires every `docking.dock_*` condition.
    forced_insertion: bool = True
    insertion_min_relative_speed_mps: float = 0.005
    insertion_max_axis_deg: float = 12.0
    insertion_max_lateral_m: float = 0.20
    # Commanded MRV acceleration limit [m/s^2] (the base is translated kinematically;
    # this limit is what keeps the 3 t payload on the arm from being yanked)
    mrv_max_accel_mps2: float = 0.004
    # v_cmd = v_client + Kp * e_mrv  (Kp [1/s]), correction capped at `max_correction_speed_mps`
    position_gain_hz: float = 0.15
    max_correction_speed_mps: float = 0.02
    # Velocity matching complete: both relative speeds below these for `stable_duration_sec`
    max_mep_client_relative_velocity_mps: float = 0.005
    max_mrv_client_relative_velocity_mps: float = 0.005
    # Rendezvous complete: |e_mrv| below this (and the velocity gates) for `stable_duration_sec`
    max_relative_position_m: float = 0.5
    stable_duration_sec: float = 1.0
    # Velocities used by the gates are least-squares slopes of the measured positions
    # over this window [s] (the MEP is dragged by a kinematically moved arm, so its
    # PhysX velocity is logged next to it but not trusted)
    velocity_window_s: float = 0.5
    # Extra docking gates while moving, next to the existing `docking.*` conditions
    dock_max_mrv_client_relative_velocity_mps: float = 0.005
    dock_max_mep_client_relative_velocity_mps: float = 0.01
    dock_max_relative_position_m: float = 0.5
    # Timeouts [s]
    chase_timeout_s: float = 30.0
    velocity_match_timeout_s: float = 90.0
    rendezvous_timeout_s: float = 90.0

    @property
    def chase_enabled(self) -> bool:
        return len(self.mrv_initial_velocity_mps) == 3

    def mrv_initial_velocity(self) -> np.ndarray:
        return np.asarray(self.mrv_initial_velocity_mps, dtype=float)


@dataclass
class PostDockingCfg:
    """`post_docking:` section -- stabilise, stop everything, release the MEP."""

    # true: DOCKED -> release the MEP at once; the docked MEP + client stack stops by
    # thrust in the background while the MRV reverses and departs (no STABILIZING /
    # STOPPING / ARM_RETREAT). false: stabilise, stop all three, release, retreat, depart.
    immediate_release: bool = True

    # STABILIZING: the docked stack keeps moving; MEP-client drift must stay small
    stabilization_sec: float = 2.0
    max_drift_mm: float = 5.0
    max_drift_deg: float = 0.5
    # STOPPING: a common deceleration a = clip(-v / tau, a_max) applied as F = m a to the
    # client AND the MEP (the MRV follows through its relative hold), so the joints
    # between them carry no extra load while the stack stops
    stop_accel_mps2: float = 0.002
    stop_time_constant_s: float = 3.0
    stop_ang_accel_rad_s2: float = 0.001
    stop_ang_time_constant_s: float = 3.0
    stop_velocity_tolerance_mps: float = 0.001
    stop_angular_tolerance_rad_s: float = 0.0005
    stop_hold_sec: float = 1.0
    stop_timeout_s: float = 90.0
    # ROBOT_RELEASE: wait after removing the capture joint before the arm moves [s]
    release_wait_sec: float = 0.5
    release_timeout_s: float = 5.0


@dataclass
class SeparationCfg:
    """`separation:` section -- arm retreat, then the MRV departs."""

    # ARM_RETREAT: back the EE off the MEP face along the face normal [m], [m/s]
    arm_retreat_distance_m: float = 0.6
    arm_retreat_speed_mps: float = 0.03
    arm_retreat_timeout_s: float = 90.0
    # Departure direction (world). [] or zero = auto: -normalize(client velocity)
    direction: List[float] = field(default_factory=list)
    velocity_mps: float = 0.02
    accel_mps2: float = 0.004
    # Cruise time at `velocity_mps` (the leg adds the accel / decel ramps around it)
    duration_sec: float = 5.0
    # The MRV-client distance must grow by at least this much [m]
    min_distance_increase_m: float = 0.1
    # SEPARATION_COLLISION above this net contact force on any arm link [N]
    max_contact_force_n: float = 50.0
    timeout_s: float = 90.0


def validate_moving_cfg(client: ClientMotionCfg, rv: RendezvousCfg, post: PostDockingCfg, sep: SeparationCfg):
    """Raise on a configuration that cannot work (called from `load_vision_config`)."""
    client.linear_velocity_mps = vec3(client.linear_velocity_mps, "client.linear_velocity_mps")
    client.angular_velocity_rad_s = vec3(client.angular_velocity_rad_s, "client.angular_velocity_rad_s")
    rv.mrv_initial_velocity_mps = ([] if rv.mrv_initial_velocity_mps in (None, [], {})
                                   else vec3(rv.mrv_initial_velocity_mps, "rendezvous.mrv_initial_velocity_mps"))
    if client.cruise_check_s < 0.0:
        raise ValueError("client.cruise_check_s must be >= 0 (0 skips CLIENT_CRUISE)")
    sep.direction = [] if sep.direction in (None, [], {}) else vec3(sep.direction, "separation.direction")
    for name, v in (("client.release_accel_mps2", client.release_accel_mps2),
                    ("client.release_ang_accel_rad_s2", client.release_ang_accel_rad_s2),
                    ("client.release_time_constant_s", client.release_time_constant_s),
                    ("rendezvous.mrv_max_accel_mps2", rv.mrv_max_accel_mps2),
                    ("rendezvous.velocity_window_s", rv.velocity_window_s),
                    ("post_docking.stop_accel_mps2", post.stop_accel_mps2),
                    ("post_docking.stop_time_constant_s", post.stop_time_constant_s),
                    ("post_docking.stop_ang_time_constant_s", post.stop_ang_time_constant_s),
                    ("separation.velocity_mps", sep.velocity_mps),
                    ("separation.accel_mps2", sep.accel_mps2),
                    ("separation.arm_retreat_speed_mps", sep.arm_retreat_speed_mps)):
        if not v > 0.0:
            raise ValueError(f"{name} must be > 0")
    if not (rv.insertion_min_relative_speed_mps > 0.0 and rv.insertion_max_axis_deg > 0.0 and rv.insertion_max_lateral_m > 0.0):
        raise ValueError("rendezvous.insertion_min_relative_speed_mps / insertion_max_axis_deg / insertion_max_lateral_m must be > 0")
    if rv.position_gain_hz < 0.0 or rv.max_correction_speed_mps < 0.0:
        raise ValueError("rendezvous.position_gain_hz and max_correction_speed_mps must be >= 0")
    if sep.arm_retreat_distance_m < 0.0 or sep.duration_sec < 0.0:
        raise ValueError("separation.arm_retreat_distance_m and duration_sec must be >= 0")
    if not client.release_enabled:
        return
    v = client.velocity()
    if float(np.linalg.norm(v)) < 1e-6:
        raise ValueError("client.linear_velocity_mps must be non-zero when client.release_enabled "
                         "(the separation direction is derived from it)")
    if float(np.linalg.norm(v)) >= client.max_speed_mps:
        raise ValueError("client.linear_velocity_mps must stay below client.max_speed_mps")
    if float(np.linalg.norm(client.angular_velocity())) >= client.max_angular_rate_rad_s:
        raise ValueError("client.angular_velocity_rad_s must stay below client.max_angular_rate_rad_s")


###########################
### Relative kinematics ###
###########################


def relative(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a - b: the state of body A relative to body B (both world-frame vectors)."""
    return np.asarray(a, dtype=float) - np.asarray(b, dtype=float)


def unit(v, fallback: Optional[np.ndarray] = None) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        if fallback is None:
            raise ValueError("cannot normalise a zero vector")
        return np.asarray(fallback, dtype=float)
    return v / n


def separation_direction(client_velocity, override: Sequence[float] = ()) -> np.ndarray:
    """Departure direction: `override` if given (non-zero), else -normalize(v_client)."""
    if override is not None and len(override) == 3 and float(np.linalg.norm(override)) > 1e-9:
        return unit(override)
    return -unit(client_velocity)


def clip_norm(v, limit: float) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v if n <= limit or n < 1e-12 else v * (limit / n)


def bounded_accel(error, time_constant: float, a_max: float) -> np.ndarray:
    """First-order closing law a = clip(error / tau, a_max) (vector norm limit)."""
    return clip_norm(np.asarray(error, dtype=float) / max(1e-6, time_constant), a_max)


class StableTimer:
    """How long a condition has held continuously (sim time); resets when it breaks."""

    def __init__(self):
        self.since: Optional[float] = None

    def update(self, ok: bool, t: float) -> float:
        if not ok:
            self.since = None
            return 0.0
        if self.since is None:
            self.since = t
        return t - self.since

    def reset(self):
        self.since = None


class VelocityEstimator:
    """Least-squares velocity of a point from its position history over `window_s`.

    Used for every velocity the rendezvous/docking gates compare, so the three bodies
    are measured the same way. A kinematically moved articulation reports link
    velocities without the base motion, and the MEP it drags through the capture joint
    may then report a wrong PhysX velocity even though its pose is right.
    """

    def __init__(self, window_s: float):
        self.window = float(window_s)
        self.t: List[float] = []
        self.p: List[np.ndarray] = []

    def reset(self):
        self.t, self.p = [], []

    def add(self, t: float, p) -> None:
        self.t.append(float(t))
        self.p.append(np.asarray(p, dtype=float).copy())
        while self.t and self.t[0] < t - self.window - 1e-9:
            self.t.pop(0)
            self.p.pop(0)

    def velocity(self) -> Optional[np.ndarray]:
        if len(self.t) < 3 or self.t[-1] - self.t[0] < 0.5 * self.window:
            return None
        t = np.asarray(self.t) - self.t[-1]
        p = np.vstack(self.p)
        tc = t - t.mean()
        return (tc[:, None] * (p - p.mean(axis=0))).sum(axis=0) / float((tc * tc).sum())


###################
### MRV control ###
###################


class MrvVelocityController:
    """Acceleration-limited velocity command for the kinematically moved MRV.

    Two modes, both ramping the command at `a_max` (the base never steps):

    - `toward(v_des)`: open loop, e.g. the chase to `mrv_initial_velocity_mps` or a stop
    - `track(...)`:    relative hold  v_des = v_client + clip(Kp * e, v_corr_max),
                       e = (p_client - p_mrv) - d_ref

    The world velocity of the MRV is never a target in itself: in `track` it is the
    client's velocity plus a correction of the RELATIVE position, so v_rel -> 0 and
    e -> 0 together.
    """

    def __init__(self, a_max: float, kp: float, v_corr_max: float):
        self.a_max = float(a_max)
        self.kp = float(kp)
        self.v_corr_max = float(v_corr_max)
        self.v = np.zeros(3)
        self.a = np.zeros(3)

    def reset(self, v0=None):
        self.v = np.zeros(3) if v0 is None else np.asarray(v0, dtype=float).copy()
        self.a = np.zeros(3)

    def toward(self, v_des, dt: float, a_max: Optional[float] = None) -> np.ndarray:
        a_lim = self.a_max if a_max is None else float(a_max)
        dv = clip_norm(np.asarray(v_des, dtype=float) - self.v, a_lim * dt)
        self.a = dv / dt if dt > 0.0 else np.zeros(3)
        self.v = self.v + dv
        return self.v.copy()

    def position_error(self, p_mrv, p_client, d_ref) -> np.ndarray:
        return relative(relative(p_client, p_mrv), d_ref)

    def track(self, p_mrv, p_client, v_client, d_ref, dt: float) -> np.ndarray:
        e = self.position_error(p_mrv, p_client, d_ref)
        v_des = np.asarray(v_client, dtype=float) + clip_norm(self.kp * e, self.v_corr_max)
        return self.toward(v_des, dt)


def matched(v_rel_mrv, v_rel_mep, max_mrv: float, max_mep: float) -> Tuple[bool, List[str]]:
    """Velocity-matching gate on RELATIVE speeds only (the world speed is irrelevant)."""
    bad = []
    for name, v, lim in (("MRV-client", v_rel_mrv, max_mrv), ("MEP-client", v_rel_mep, max_mep)):
        s = math.inf if v is None else float(np.linalg.norm(v))
        if not s <= lim:
            bad.append(f"{name} relative velocity {s*1000:.2f} mm/s > {lim*1000:.1f} mm/s")
    return not bad, bad


def stopped(speeds: Sequence[Optional[float]], rates: Sequence[Optional[float]], v_tol: float, w_tol: float) -> bool:
    """All world speeds below `v_tol` and all angular rates below `w_tol`."""
    return all(s is not None and s <= v_tol for s in speeds) and all(w is not None and w <= w_tol for w in rates)


def insertion_corridor_ok(lateral_m: float, axis_deg: float, clearance_m: float, max_lateral_m: float,
                          max_axis_deg: float, min_clearance_m: float) -> Tuple[bool, List[str]]:
    """Forced insertion keeps advancing inside this corridor (relative quantities only).

    `clearance_m` is the probe-to-nozzle-wall gap at the current depth (inf outside the
    nozzle); it is the physical limit that stays even when the corridor is widened."""
    bad = []
    if not lateral_m <= max_lateral_m:
        bad.append(f"lateral {lateral_m*1000:.1f} mm > {max_lateral_m*1000:.0f} mm")
    if not axis_deg <= max_axis_deg:
        bad.append(f"axis {axis_deg:.2f} deg > {max_axis_deg:g}")
    if not clearance_m >= min_clearance_m:
        bad.append(f"wall clearance {clearance_m*1000:.0f} mm < {min_clearance_m*1000:.0f} mm")
    return not bad, bad


def separation_leg_length(velocity: float, accel: float, cruise_s: float) -> float:
    """Length of a trapezoidal leg that cruises `cruise_s` at `velocity` (ramps at `accel`)."""
    return velocity * cruise_s + velocity * velocity / accel


###############
### Logging ###
###############


def _xyz(prefix: str) -> Tuple[str, str, str]:
    return tuple(f"{prefix}_{a}" for a in "xyz")


# `<run>_moving.csv`, one row per `logging.rate_hz` tick from CLIENT_RELEASE on.
# World frame, SI units. `*_velocity_*` of the client / MEP are the least-squares slopes
# the gates use; `*_physx_velocity_*` are the simulator's own body velocities, and the
# MRV velocity is the commanded kinematic velocity (`mrv_measured_velocity_*`: from poses).
# `mep_client_relative_*` is MEP docking point (probe tip) minus client docking point
# (SAT_DOCK_POINT); `mrv_station_error` is e = (p_client - p_mrv) - d_ref.
CSV_COLUMNS: Tuple[str, ...] = (
    "run_id", "timestamp", "state",
    *_xyz("client_position"), *_xyz("client_velocity"), *_xyz("client_physx_velocity"), *_xyz("client_angular_velocity"),
    *_xyz("mep_position"), *_xyz("mep_velocity"), *_xyz("mep_physx_velocity"), *_xyz("mep_angular_velocity"),
    *_xyz("mrv_position"), *_xyz("mrv_velocity"), *_xyz("mrv_measured_velocity"), *_xyz("mrv_acceleration"),
    *_xyz("mep_client_relative_position"), "mep_client_relative_position",
    *_xyz("mep_client_relative_velocity"), "mep_client_relative_velocity",
    *_xyz("mrv_client_relative_position"), "mrv_client_relative_position",
    *_xyz("mrv_station_error"), "mrv_station_error",
    *_xyz("mrv_client_relative_velocity"), "mrv_client_relative_velocity",
    "docking_position_error", "docking_orientation_error",
    "robot_mep_attached", "mep_client_docked",
    "mrv_client_distance", "separation_distance",
    *_xyz("client_force"), *_xyz("mep_force"), "arm_contact_force",
)
