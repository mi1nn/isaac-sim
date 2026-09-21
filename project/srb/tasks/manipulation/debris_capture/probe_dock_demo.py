"""Probe-into-thruster docking driven by GPS telemetry and a probe-tip depth camera.

The method, in one line: **the topic says which way, the camera says how far.**

    ALIGN   the thruster pose arrives over ROS 2 (noisy, late, biased). Build the error
            in the probe tip frame, take its Euler angles, and drive the tip onto the
            thruster axis until the two axes are parallel and concentric.
    INSERT  slide along that axis. The remaining depth comes from the depth camera, not
            from the topic, and the topic keeps arriving so the axis can be re-corrected
            while moving.
    DOCK    at the commanded depth, with the camera and the topic agreeing, create the
            fixed joint.

Why the split. The GPS link carries a slowly wandering bias (`gps_link.py`) that no
amount of averaging removes. A few centimetres of it sideways is harmless -- the bore is
0.93 m wide at the dock point and the probe is 0.16 m across. The same few centimetres
*along* the axis is the difference between docking and hitting the back of the thruster.
So the axial number is the one measurement that never comes from the topic.

The MEP is already attached to the arm when this starts (the arm carries it, as in
`docking_demo.py`); this demo owns the satellite-side half of the sequence.
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch

from .frames import Frame, rotation_angle
from .gps_link import GpsLinkCfg, GpsSample, make_gps_link
from .vision import rotmat_to_rpy_deg


class State(Enum):
    INIT = "INIT"
    WAIT_TELEMETRY = "WAIT_TELEMETRY"
    ALIGN = "ALIGN"
    INSERT = "INSERT"
    DOCK = "DOCK"
    DOCKED = "DOCKED"
    UNDOCKED = "UNDOCKED"
    # failures
    TELEMETRY_LOST = "TELEMETRY_LOST"
    ALIGN_TIMEOUT = "ALIGN_TIMEOUT"
    INSERT_TIMEOUT = "INSERT_TIMEOUT"
    DEPTH_INVALID = "DEPTH_INVALID"
    WALL_CONTACT = "WALL_CONTACT"
    DOCK_REJECTED = "DOCK_REJECTED"


FAILURES = {
    State.TELEMETRY_LOST,
    State.ALIGN_TIMEOUT,
    State.INSERT_TIMEOUT,
    State.DEPTH_INVALID,
    State.WALL_CONTACT,
    State.DOCK_REJECTED,
}
TERMINAL = FAILURES | {State.DOCKED, State.UNDOCKED}
MOVING = {State.ALIGN, State.INSERT}


@dataclass
class ProbeDockCfg:
    """Tolerances and speeds. Distances in m, angles in deg unless stated."""

    gps: GpsLinkCfg = field(default_factory=GpsLinkCfg)
    # Overrides applied to `DockingDemo`'s `DemoCfg` for this demo only -- the GT demo
    # keeps its validated Phase-1 values. Any `DemoCfg` field name is accepted; the
    # loader rejects names that do not exist, so a typo is not silently ignored.
    # Note the arm is drive-limited: `run_segment` only advances its target while the
    # tracking error stays under `lag_pos`, so raising a speed past what the joints can
    # follow buys nothing. Measure before believing a number here.
    arm: Dict[str, float] = field(default_factory=dict)

    # Stand-off in front of the nozzle exit where the alignment is judged (negative
    # depth = outside the bore)
    approach_depth_m: float = -1.0
    # Commanded final insertion depth. `DockingCfg.dock_depth` is the design value.
    dock_depth_m: float = 0.8

    # ALIGN gates, all measured in the probe tip frame against the telemetry pose
    align_radial_m: float = 0.02
    align_axis_deg: float = 0.5
    align_roll_deg: float = 1.0
    # The telemetry is noisy, so the gates are judged on a smoothed estimate
    telemetry_filter_tau_s: float = 1.0
    # A pose older than this means the link is gone
    telemetry_timeout_s: float = 2.0

    # Speeds
    align_speed_mps: float = 0.10
    align_speed_ang_deg_s: float = 6.0
    insert_speed_mps: float = 0.04
    # Below this remaining depth the insertion creeps
    creep_range_m: float = 0.15
    insert_creep_mps: float = 0.015
    max_joint_step_rad: float = 0.06
    ik_lambda: float = 0.05

    # Depth camera
    depth_valid_min_m: float = 0.05
    depth_filter_tau_s: float = 0.3
    # Reject a frame whose centre patch disagrees with the filtered value by more
    depth_outlier_m: float = 0.25
    # Optical centring from the depth ring. This carries the lateral precision;
    # `insert_max_radial_m` below is only a gross-excursion guard.
    centring_enabled: bool = True
    # Metres of lateral command per metre of range difference across the ring
    centring_gain: float = 0.35
    centring_max_step_m: float = 0.05
    # Ring spread below which the bore looks symmetric enough to dock
    centring_docked_span_m: float = 0.05
    # Gross-excursion guard against the *believed* axis. The telemetry is ~100 mm out so
    # this cannot be tight; the real safety net is `min_wall_clearance_m`. At the dock
    # point the bore is 466 mm and the probe 80 mm, leaving 386 mm of clearance.
    insert_max_radial_m: float = 0.30
    # Stop if the modelled probe-to-wall clearance drops below this
    min_wall_clearance_m: float = 0.05

    # DOCK gates
    dock_depth_tol_m: float = 0.02
    # Against the believed axis -- loose, because the belief is what is uncertain. The
    # tight number that actually matters is `centring_docked_span_m` (optical).
    dock_radial_m: float = 0.15
    dock_axis_deg: float = 0.5

    # Timeouts [s of simulated time]
    # The transport leg carries the MEP ~5.5 m at `speed_carry`; give it room
    approach_timeout_s: float = 240.0
    # Arrival at the (frozen) stand-off station. Coarse on purpose: the telemetry this
    # station was derived from is itself tens of mm out, so tighter numbers here would
    # just be chasing the noise. ALIGN then closes on the live belief.
    approach_arrive_m: float = 0.05
    approach_arrive_deg: float = 1.0
    approach_arrive_speed_mps: float = 0.005
    # How long a headless run lingers in a terminal state before exiting. The GUI
    # ignores this and idles until the window is closed.
    terminal_hold_s: float = 3.0
    align_timeout_s: float = 120.0
    insert_timeout_s: float = 120.0
    settle_steps: int = 3


def euler_error_deg(err: Frame) -> Tuple[float, float, float]:
    """Roll/pitch/yaw [deg] of a pose error, as the alignment is specified in."""
    return rotmat_to_rpy_deg(err.rot)


class DepthEstimator:
    """Turns the probe camera's depth image into an insertion depth.

    Only the centre patch is used for the axial number: it is the part of the image
    looking straight down the bore. Pixels at the near plane are dropped -- they mean
    the ray hit nothing valid, not that a surface is 2 cm away.
    """

    def __init__(self, cfg: ProbeDockCfg, bore, camera_cfg):
        self.cfg = cfg
        self.bore = bore
        self.cam = camera_cfg
        self.value: Optional[float] = None
        self.raw: Optional[float] = None
        self.last_t: Optional[float] = None
        self.rejected = 0
        self.frames = 0
        # Ring asymmetry, in camera axes: (+x right, +y down), metres of range
        # difference between opposite sides. Zero when the probe is on the bore axis.
        self.balance: Optional[Tuple[float, float]] = None
        self.ring_span: float = 0.0

    def _ring_balance(self, image: np.ndarray, near: float):
        """How lopsided the bore looks -- the optical centring signal.

        A ray that is off-axis meets the converging wall sooner than one on the axis, so
        the side the probe has drifted towards reads *shorter*. Differencing opposite
        sides of a ring of pixels gives a null-seeking error signal that needs no
        calibration and, unlike the telemetry, is not biased: it is zero exactly when
        the probe is centred in the bore it is actually looking at.
        """
        h, w = image.shape[:2]
        cy, cx = h / 2.0, w / 2.0
        r = float(self.cam.ring_fraction) * min(h, w) / 2.0
        band = max(2.0, 0.08 * min(h, w))
        yy, xx = np.mgrid[0:h, 0:w]
        rad = np.hypot(yy - cy, xx - cx)
        on_ring = (np.abs(rad - r) <= band) & np.isfinite(image) & (image > near * 1.05)
        if on_ring.sum() < 16:
            return None, 0.0

        def side(mask):
            sel = on_ring & mask
            return float(np.median(image[sel])) if sel.sum() >= 4 else None

        left, right = side(xx < cx - 0.3 * r), side(xx > cx + 0.3 * r)
        up, down = side(yy < cy - 0.3 * r), side(yy > cy + 0.3 * r)
        if None in (left, right, up, down):
            return None, 0.0
        vals = [v for v in (left, right, up, down)]
        return (left - right, up - down), float(max(vals) - min(vals))

    def update(self, t: float, image: np.ndarray, radial: float = 0.0) -> Optional[float]:
        self.frames += 1
        h, w = image.shape[:2]
        near = float(self.cam.clipping_range_m[0])
        k = max(2, min(h, w) // 40)
        patch = image[h // 2 - k : h // 2 + k + 1, w // 2 - k : w // 2 + k + 1]
        good = patch[np.isfinite(patch) & (patch > near * 1.05)]
        if good.size == 0:
            return self.value
        # Nearest surface straight ahead, robust to a few stray pixels
        rng = float(np.percentile(good, 20.0))
        self.balance, self.ring_span = self._ring_balance(image, near)
        # The camera sits `mount_offset_m` in front of the tip face. `radial` says which
        # surface the centre ray is really hitting -- on-axis it is the end of the bore,
        # off-axis it is the wall, and the difference is decimetres.
        depth = self.bore.depth_from_range(rng + self.cam.mount_offset_m, radial)
        self.raw = depth
        if self.value is None:
            self.value = depth
        elif abs(depth - self.value) > self.cfg.depth_outlier_m:
            self.rejected += 1
        else:
            dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
            a = 1.0 - math.exp(-dt / max(1e-6, self.cfg.depth_filter_tau_s)) if dt > 0 else 1.0
            self.value += a * (depth - self.value)
        self.last_t = t
        return self.value


class TelemetryFilter:
    """Low-pass on the received thruster pose: kills the white noise, keeps the bias."""

    def __init__(self, tau_s: float):
        self.tau = tau_s
        self.pose: Optional[Frame] = None
        self.stamp: Optional[float] = None
        self.count = 0

    def update(self, sample: GpsSample) -> Frame:
        self.count += 1
        if self.pose is None or self.stamp is None:
            self.pose, self.stamp = sample.pose, sample.stamp
            return self.pose
        dt = max(0.0, sample.stamp - self.stamp)
        a = 1.0 - math.exp(-dt / max(1e-6, self.tau)) if dt > 0 else 0.0
        pos = self.pose.pos + a * (sample.pose.pos - self.pose.pos)
        # Rotation: step a fraction of the way along the relative rotation
        rel = self.pose.rot.T @ sample.pose.rot
        ang = rotation_angle(np.eye(3), rel)
        if ang > 1e-9:
            axis_m = 0.5 * (rel - rel.T)
            axis = np.array([axis_m[2, 1], axis_m[0, 2], axis_m[1, 0]])
            n = float(np.linalg.norm(axis))
            if n > 1e-12:
                axis /= n
                rot = self.pose.rot @ _rotmat_axis_angle(axis, a * ang)
            else:
                rot = sample.pose.rot
        else:
            rot = self.pose.rot
        self.pose, self.stamp = Frame(pos, rot), sample.stamp
        return self.pose


# `DemoCfg` keeps these in radians; the YAML states them in degrees. Listed explicitly
# rather than matched by suffix -- `lag_ang_precise` ends in neither "_ang" nor
# "_angle" and was being written straight through as radians.
_DEMO_ANGLE_FIELDS = frozenset({
    "speed_ang", "lag_ang", "lag_ang_precise", "settle_ang", "settle_ang_carry",
    "settle_window_ang", "attach_normal_angle", "attach_roll_angle",
    "attach_approach_angle", "dock_axis_angle", "dock_roll_angle",
    "dock_approach_angle", "insert_max_axis",
})


def _demo_cfg_with(overrides: Dict[str, float]):
    """A `DemoCfg` with `overrides` applied, rejecting names it does not have."""
    from .docking_demo import DemoCfg

    cfg = DemoCfg()
    applied = {}
    for key, value in (overrides or {}).items():
        if not hasattr(cfg, key):
            raise KeyError(
                f"Unknown arm override 'arm.{key}'. Valid names are the fields of "
                f"DemoCfg in docking_demo.py."
            )
        setattr(cfg, key, math.radians(value) if key in _DEMO_ANGLE_FIELDS else value)
        applied[key] = value
    if applied:
        print(f"[ARM] DemoCfg overrides: {applied}", flush=True)
    return cfg


def _rotmat_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


###############################
### The demo (state machine) ##
###############################

from .docking_demo import DockingDemo  # noqa: E402
from .docking_demo import State as DockState  # noqa: E402
from .docking_demo import fail, interp_frame, pose_error  # noqa: E402

# The states this demo takes over. Everything before them -- planning, approaching the
# MEP, the 6-DoF grasp alignment, the attach and the lift -- is the servicer handling its
# own arm and its own payload, where forward kinematics is the honest source of truth.
# From here on the target belongs to somebody else, and the only things we may read are
# the telemetry topic and the probe camera.
SATELLITE_SIDE = {
    DockState.MOVE_TO_SAT_PREDOCK,
    DockState.ALIGN_SAT,
    DockState.INSERT_PROBE,
    DockState.DOCK,
}


class ProbeDockDemo(DockingDemo):
    """`DockingDemo` with the satellite half flown on telemetry + the probe camera."""

    def __init__(self, env, sim_app, headless: bool, cfg=None, dock_cfg: Optional[ProbeDockCfg] = None,
                 use_ros: bool = True, auto_undock_after=None, exit_after=None):
        pcfg = dock_cfg or ProbeDockCfg()
        cfg = cfg or _demo_cfg_with(pcfg.arm)
        super().__init__(env, sim_app, headless, cfg, auto_undock_after, exit_after)
        self.pcfg = pcfg
        self.pcfg.dock_depth_m = float(self.task.cfg.docking.dock_depth)
        self.bore = self.task.bore
        self.pcam = self.scene[self.task.cfg.probe_camera.name]
        self.pcam_cfg = self.task.cfg.probe_camera
        self.link = make_gps_link(self.pcfg.gps, use_ros=use_ros)
        self.depth = DepthEstimator(self.pcfg, self.bore, self.pcam_cfg)
        self.telemetry = TelemetryFilter(self.pcfg.telemetry_filter_tau_s)
        self.belief: Optional[Frame] = None  # filtered thruster dock frame, as received
        self.last_sample: Optional[GpsSample] = None
        self._ref: Optional[Frame] = None
        self._depth_cmd: Optional[float] = None
        self._viewport_switched = False
        self._view_window = None
        self._timing: List[Tuple[str, float, float]] = []
        self._state_started = (0.0, time.time())
        self._wall_start = time.time()
        self._insert_max_radial = 0.0
        self._min_wall = math.inf
        self._telemetry_err: List[Tuple[float, float]] = []
        self._depth_err: List[float] = []
        self.probe_results: Dict[str, bool] = {}

    ## Ground truth -- only the client (the publisher) is allowed to touch this
    def thruster_gt(self) -> Frame:
        return self.docking.dock_world()

    def sat_gt(self) -> Frame:
        return self.docking.sat_frame()

    def true_depth(self) -> float:
        """Insertion depth of the probe tip, from physics. Logging and checks only."""
        dock = self.thruster_gt()
        axis = dock.rot[:, 2]
        return float((self.docking.probe_world().pos - dock.pos) @ axis) + self.pcfg.dock_depth_m

    def true_radial(self) -> float:
        dock = self.thruster_gt()
        axis = dock.rot[:, 2]
        rel = self.docking.probe_world().pos - dock.pos
        return float(np.linalg.norm(rel - (rel @ axis) * axis))

    ## Sensing
    def _telemetry_tick(self):
        self.link.publish(self.sim_time, self.sat_gt(), self.thruster_gt())
        self.link.spin(self.sim_time)
        s = self.link.latest(self.sim_time)
        if s is not None and (self.last_sample is None or s.stamp > self.last_sample.stamp):
            self.last_sample = s
            self.belief = self.telemetry.update(s)
            gt = self.thruster_gt()
            self._telemetry_err.append(
                (float(np.linalg.norm(self.belief.pos - gt.pos)), math.degrees(rotation_angle(self.belief.rot, gt.rot)))
            )

    def _depth_tick(self):
        out = self.pcam.data.output
        if "distance_to_image_plane" not in out:
            return
        img = out["distance_to_image_plane"][0].cpu().numpy().squeeze()
        before = self.depth.value
        self.depth.update(self.sim_time, np.asarray(img, dtype=float))
        if self.depth.value is not None and before is not None:
            self._depth_err.append(self.depth.value - self.true_depth())

    ## Geometry in the believed frame
    def believed_tip_goal(self, depth: float) -> Frame:
        """Tip pose on the believed bore axis at `depth` from the nozzle exit plane."""
        t = self.belief
        assert t is not None
        return Frame(t.pos + (depth - self.pcfg.dock_depth_m) * t.rot[:, 2], t.rot)

    def alignment_error(self) -> Dict[str, float]:
        """Probe tip vs the believed dock frame, as the operator specified it: radial
        offset from the axis, and the roll/pitch/yaw of the orientation error."""
        t, tip = self.belief, self.tool()
        if t is None:
            return {k: math.nan for k in ("radial", "axial", "roll", "pitch", "yaw", "axis")}
        axis = t.rot[:, 2]
        rel = tip.pos - t.pos
        axial = float(rel @ axis)
        radial = float(np.linalg.norm(rel - axial * axis))
        err = Frame(np.zeros(3), t.rot.T @ tip.rot)
        roll, pitch, yaw = euler_error_deg(err)
        return {
            "radial": radial,
            "axial": axial + self.pcfg.dock_depth_m,  # as a depth
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "axis": math.degrees(math.acos(max(-1.0, min(1.0, float(tip.rot[:, 2] @ axis))))),
        }

    ## Motion
    def track_to(self, goal: Frame, speed: float, ang_speed_deg: float):
        """Rate-limited servo of the probe tip towards `goal` (which moves with the
        telemetry), then one damped-least-squares IK step, as elsewhere in this repo."""
        if self._ref is None:
            self._ref = self.tool()
        lag_p, lag_a = pose_error(self.tool(), self._ref)
        pos, rot = self._ref.pos.copy(), self._ref.rot
        if lag_p < 0.05 and lag_a < math.radians(3.0):
            d = goal.pos - pos
            n = float(np.linalg.norm(d))
            if n > 1e-9:
                pos = pos + d / n * min(n, speed * self.dt)
            ang = rotation_angle(rot, goal.rot)
            if ang > 1e-9:
                s = min(1.0, math.radians(ang_speed_deg) * self.dt / ang)
                rot = interp_frame(Frame(pos, rot), Frame(pos, goal.rot), s).rot
        self._ref = Frame(pos, rot)
        q = self.arm.ik_joint_target(self._ref, self.pcfg.max_joint_step_rad)
        self.task._robot.set_joint_position_target(q, joint_ids=self.arm.joint_ids)

    ## GUI
    def open_camera_window(self):
        """Open the separate camera window, starting on the wrist view of the capture."""
        if self.headless:
            return
        try:
            from omni.kit.viewport.utility import create_viewport_window
            from pxr import Sdf

            self._view_window = create_viewport_window(
                "cam_wrist -> cam_probe", width=640, height=360, position_x=40, position_y=60,
            )
            # Set the camera through the API rather than the constructor: `camera_path=`
            # was landing on the wrong camera, and this way the window reports back which
            # prim it actually ended up on, so a mismatch is visible instead of silent.
            self._view_window.viewport_api.camera_path = Sdf.Path(self.task.wrist_camera_path)
            shown = str(self._view_window.viewport_api.camera_path)
            print(f"[VIEW] camera window on {shown}", flush=True)
            if shown != self.task.wrist_camera_path:
                print(f"[VIEW] WARNING: asked for {self.task.wrist_camera_path}, got {shown}", flush=True)
        except Exception as e:  # a GUI convenience must never stop the run
            print(f"[VIEW] could not open the camera window: {e} -- pick 'cam_wrist' in the camera menu", flush=True)

    def switch_viewport_to_probe(self):
        """Repoint that window at the probe's RGB camera, once the MEP is on the arm."""
        if self._viewport_switched or self.headless:
            self._viewport_switched = True
            return
        self._viewport_switched = True
        path = self.task.probe_camera_path
        try:
            from pxr import Sdf

            if self._view_window is not None:
                self._view_window.viewport_api.camera_path = Sdf.Path(path)
                shown = str(self._view_window.viewport_api.camera_path)
                print(f"[VIEW] camera window -> {shown} (the probe's RGB view of the insertion)", flush=True)
            else:
                from omni.kit.viewport.utility import get_active_viewport

                vp = get_active_viewport()
                vp.camera_path = path
                print(f"[VIEW] main viewport -> {path}", flush=True)
        except Exception as e:
            print(f"[VIEW] could not switch to the probe camera ({e}); pick 'cam_probe' in the camera menu", flush=True)

    def run(self):
        self.open_camera_window()
        self._wall_start = time.time()
        try:
            return super().run()
        finally:
            self._report_timing()

    def goto(self, state):
        """Record how long each state took, in both clocks."""
        now = time.time()
        prev = getattr(self, "_state_started", None)
        if prev is not None:
            self._timing.append((self.state.name, self.sim_time - prev[0], now - prev[1]))
        self._state_started = (self.sim_time, now)
        super().goto(state)

    def _report_timing(self):
        if not self._timing:
            return
        sim_total = sum(s for _, s, _ in self._timing)
        wall_total = sum(w for _, _, w in self._timing)
        print("[TIMING] per state: simulated / wall seconds", flush=True)
        for name, s, w in self._timing:
            print(f"[TIMING]   {name:<22} {s:7.1f} s sim   {w:7.1f} s wall", flush=True)
        rtf = sim_total / wall_total if wall_total > 0 else float("nan")
        print(f"[TIMING]   {'TOTAL':<22} {sim_total:7.1f} s sim   {wall_total:7.1f} s wall "
              f"(real-time factor {rtf:.2f}x)", flush=True)

    ## Stepping
    ## Terminal states park the arm and hold. In the GUI that is the whole point -- the
    ## window must outlive the sequence. Headless has no window and nothing to look at,
    ## so it has to end: without this the process span forever on FAILED, and three
    ## abandoned runs once ate 11 GB of GPU memory between them.
    TERMINAL_STATES = (DockState.DOCKED, DockState.FAILED, DockState.RELEASED, DockState.UNDOCKED)

    def _finish_if_headless(self):
        if self.headless and self.state in self.TERMINAL_STATES and self.state_time >= self.pcfg.terminal_hold_s:
            if not self.done:
                print(f"[DEMO] terminal state {self.state.name} held {self.pcfg.terminal_hold_s:.1f} s "
                      f"-- ending the headless run", flush=True)
            self.done = True

    def step(self):
        if self.state not in SATELLITE_SIDE:
            super().step()
            self._telemetry_tick()  # keep the link running so the filter and bias are warm
            self._finish_if_headless()
            return
        self.state_time += self.dt
        self.sim_time += self.dt
        self._telemetry_tick()
        self._depth_tick()
        self.switch_viewport_to_probe()

        if self.belief is None or (self.sim_time - (self.last_sample.received if self.last_sample else 0.0)) > self.pcfg.telemetry_timeout_s:
            if self.state != DockState.MOVE_TO_SAT_PREDOCK or self.belief is None:
                self.hold()
                if self.state_time > self.pcfg.telemetry_timeout_s:
                    fail(f"no thruster telemetry for {self.pcfg.telemetry_timeout_s:.1f} s")
                    self.goto(DockState.FAILED)
                return

        s = self.state
        if s == DockState.MOVE_TO_SAT_PREDOCK:
            self._step_approach()
        elif s == DockState.ALIGN_SAT:
            self._step_align()
        elif s == DockState.INSERT_PROBE:
            self._step_insert()
        elif s == DockState.DOCK:
            self._step_dock()
        self._log_probe()
        self._finish_if_headless()

    def _step_approach(self):
        """Carry the MEP the ~5.5 m to the believed stand-off.

        This leg uses the parent's segment machinery, not the servo below it: it is a
        long straight transport of a 3 t payload and `run_segment` already has the
        acceleration profile, the lag handling and the settle test for exactly that.
        The only change from the GT demo is where the goal comes from -- the telemetry
        instead of the satellite's real pose. The belief is re-read each step, so a
        client that drifts is still followed.
        """
        if self.segment is None:
            # Freeze the goal for the length of the segment. Re-aiming it every step at
            # the live belief looked more responsive but could never finish: the
            # telemetry carries 3 cm of white noise, `run_segment`'s carrying settle
            # tolerance is 5 mm, and a goal that jitters by more than the tolerance is
            # never "reached". Measured: the tip parked 103 mm out and timed out. The
            # station only has to be approximately right -- ALIGN does the fine work
            # against the live belief afterwards.
            self._approach_goal = self.believed_tip_goal(self.pcfg.approach_depth_m)
            self.start_segment("transport (telemetry)", self._approach_goal, self.cfg.speed_carry)
        if not self.check_contacts(True):
            fail("unexpected contact while carrying the MEP to the stand-off")
            return self.goto(DockState.FAILED)
        settled = self.run_segment()
        # Arrival against the frozen goal: the belief itself is tens of mm out, so
        # demanding millimetre settling here would be measuring the noise, not the arm.
        pe, ae = pose_error(self.tool(), self._approach_goal)
        v_lin, _ = self.arm.tool_velocity()
        arrived = bool(settled) or (
            self.segment is not None and self.segment.u >= 1.0
            and pe < self.pcfg.approach_arrive_m
            and math.degrees(ae) < self.pcfg.approach_arrive_deg
            and v_lin < self.pcfg.approach_arrive_speed_mps
        )
        if arrived:
            m = self.alignment_error()
            self.record("Probe reached the telemetry stand-off", True,
                        f"depth {m['axial']:.3f} m, radial {m['radial']*1000:.1f} mm, "
                        f"telemetry error {self._telemetry_err[-1][0]*1000:.1f} mm")
            self._ref = None
            self.goto(DockState.ALIGN_SAT)
        elif self.state_time > self.pcfg.approach_timeout_s:
            m = self.alignment_error()
            fail(f"stand-off not reached (depth {m['axial']:.3f} m, radial {m['radial']*1000:.0f} mm)")
            self.goto(DockState.FAILED)

    def _step_align(self):
        p = self.pcfg
        self.track_to(self.believed_tip_goal(p.approach_depth_m), p.align_speed_mps, p.align_speed_ang_deg_s)
        if self.state_time < 1e-9:
            self._settled = 0
        m = self.alignment_error()
        ok = (m["radial"] < p.align_radial_m and m["axis"] < p.align_axis_deg
              and abs(m["roll"]) < p.align_roll_deg and abs(m["axial"] - p.approach_depth_m) < 0.02)
        if ok:
            self._settled = getattr(self, "_settled", 0) + 1
        else:
            self._settled = 0
        if self._settled >= p.settle_steps:
            d = self.depth.value
            self.record(
                "Probe/thruster alignment on telemetry", True,
                f"radial {m['radial']*1000:.1f} mm, axis {m['axis']:.3f} deg, "
                f"euler (r,p,y) ({m['roll']:.3f}, {m['pitch']:.3f}, {m['yaw']:.3f}) deg | "
                f"camera depth {d:.3f} m vs GT {self.true_depth():.3f} m",
            )
            if d is None or d < -3.0:
                fail(f"no usable depth reading at the stand-off ({d})")
                return self.goto(DockState.FAILED)
            self._depth_cmd = d
            self._insert_max_radial, self._min_wall = 0.0, math.inf
            self._settled = 0
            self.goto(DockState.INSERT_PROBE)
        elif self.state_time > p.align_timeout_s:
            fail(f"alignment not achieved (radial {m['radial']*1000:.1f} mm, axis {m['axis']:.3f} deg)")
            self.goto(DockState.FAILED)

    def _step_insert(self):
        p = self.pcfg
        d_meas = self.depth.value
        if d_meas is None:
            self.hold()
            return
        remaining = p.dock_depth_m - d_meas
        speed = p.insert_creep_mps if remaining < p.creep_range_m else p.insert_speed_mps
        # The command follows the *camera*: this is the one number the telemetry bias
        # would ruin, and the bore's back plate is only 100 mm past the dock point.
        self._depth_cmd = min(p.dock_depth_m, self._depth_cmd + speed * self.dt)
        m = self.alignment_error()
        # Lateral and angular corrections keep coming from the telemetry while moving
        goal = self.believed_tip_goal(self._depth_cmd)
        axis = self.belief.rot[:, 2]
        # Axial part of the goal is servoed on the measured depth, not the believed one
        goal = Frame(goal.pos + axis * (self._depth_cmd - d_meas - (m["axial"] - self._depth_cmd)), goal.rot)
        self.track_to(goal, max(speed * 3.0, 0.02), p.align_speed_ang_deg_s)

        if d_meas > 0.0:
            self._insert_max_radial = max(self._insert_max_radial, m["radial"])
            self._min_wall = min(self._min_wall, self.bore.wall_clearance(d_meas, m["radial"]))
            if m["radial"] > p.insert_max_radial_m:
                fail(f"probe strayed {m['radial']*1000:.0f} mm off the believed axis while inside the bore")
                return self.goto(DockState.FAILED)
            if self._min_wall < p.min_wall_clearance_m:
                fail(f"modelled probe-to-wall clearance {self._min_wall*1000:.0f} mm below the limit")
                return self.goto(DockState.FAILED)
        if abs(d_meas - p.dock_depth_m) < p.dock_depth_tol_m and self._depth_cmd >= p.dock_depth_m - 1e-9:
            self.record("Probe insertion stayed on the axis", True,
                        f"max radial {self._insert_max_radial*1000:.1f} mm, min modelled wall clearance {self._min_wall*1000:.0f} mm")
            self.goto(DockState.DOCK)
        elif self.state_time > p.insert_timeout_s:
            fail(f"insertion stalled at camera depth {d_meas:.3f} m (target {p.dock_depth_m:.3f} m)")
            self.goto(DockState.FAILED)

    def _step_dock(self):
        p = self.pcfg
        self.hold()
        d_meas = self.depth.value
        m = self.alignment_error()
        gt_depth, gt_radial = self.true_depth(), self.true_radial()
        detail = (f"camera depth {d_meas:.4f} m (GT {gt_depth:.4f} m, error {abs(d_meas-gt_depth)*1000:.1f} mm), "
                  f"radial {m['radial']*1000:.1f} mm (GT {gt_radial*1000:.1f} mm), axis {m['axis']:.3f} deg, "
                  f"euler (r,p,y) ({m['roll']:.3f}, {m['pitch']:.3f}, {m['yaw']:.3f}) deg")
        ok = (abs(d_meas - p.dock_depth_m) < p.dock_depth_tol_m and m["radial"] < p.dock_radial_m
              and m["axis"] < p.dock_axis_deg)
        if ok:
            self.record("Docking conditions (telemetry + camera)", True, detail)
            self.docking.dock()
            self.record("Satellite docking", self.docking.is_docked, "fixed joint MEP<->satellite created")
            self.docked_time = self.sim_time
            self.goto(DockState.DOCKED)
        elif self.state_time > 3.0:
            self.record("Docking conditions (telemetry + camera)", False, detail)
            fail("docking conditions not met; not docking")
            self.goto(DockState.FAILED)

    def _log_probe(self):
        if self.sim_time - getattr(self, "_probe_last_log", -1.0) < 1.0:
            return
        self._probe_last_log = self.sim_time
        m = self.alignment_error()
        d = self.depth.value
        te = self._telemetry_err[-1] if self._telemetry_err else (math.nan, math.nan)
        b = self.depth.balance
        bal = f"{b[0]*1000:+6.0f},{b[1]*1000:+6.0f}" if b else "    -,     -"
        print(
            f"[PDOCK] t={self.sim_time:6.1f}s {self.state.name:<20} "
            f"depth cam {d if d is not None else float('nan'):7.3f} GT {self.true_depth():7.3f} | "
            f"radial(belief) {m['radial']*1000:6.1f} GT {self.true_radial()*1000:6.1f} mm | "
            f"ring {bal} span {self.depth.ring_span*1000:5.0f} mm | "
            f"tlm err {te[0]*1000:5.1f} mm {te[1]:4.2f} deg",
            flush=True,
        )

    def report_sensors(self):
        """How well each sensor did -- the point of the whole exercise."""
        if self._telemetry_err:
            pos = [p for p, _ in self._telemetry_err]
            ang = [a for _, a in self._telemetry_err]
            print(
                f"[SENSOR] telemetry: {len(pos)} poses, position error mean "
                f"{np.mean(pos)*1000:.1f} mm / max {np.max(pos)*1000:.1f} mm, "
                f"attitude mean {np.mean(ang):.2f} deg / max {np.max(ang):.2f} deg "
                f"(published {self.link.published}, dropped {self.link.dropped})",
                flush=True,
            )
        if self._depth_err:
            e = np.abs(self._depth_err)
            print(
                f"[SENSOR] depth camera: {self.depth.frames} frames, insertion-depth error mean "
                f"{np.mean(e)*1000:.1f} mm / max {np.max(e)*1000:.1f} mm, {self.depth.rejected} outliers rejected",
                flush=True,
            )
        if self._telemetry_err and self._depth_err:
            pos = np.array([p for p, _ in self._telemetry_err])
            print(
                f"[SENSOR] this is the trade the design rests on: telemetry is "
                f"{np.mean(pos)*1000:.0f} mm out on average and is only trusted for direction; "
                f"the camera is {np.mean(np.abs(self._depth_err))*1000:.1f} mm out and carries the axial depth.",
                flush=True,
            )

    def idle_forever(self):
        """GUI: keep simulating (joints held) until the person closes the window.

        Reaching a milestone -- docked, failed, or the control loop raising -- is never
        a reason to tear the application down. Anything that goes wrong in here is
        swallowed for the same reason.
        """
        try:
            self.q_hold = self.arm.joint_pos().clone()
        except Exception:
            self.q_hold = None
        outcome = "DOCKED" if self.docking.is_docked else self.state.value
        print(
            f"[DEMO] finished in {outcome} -- the simulation keeps running. "
            f"The viewport is on the probe RGB camera; close the window to exit.",
            flush=True,
        )
        render_interval = max(1, self.task.cfg.sim.render_interval)
        n = 0
        with torch.no_grad():
            while self.sim_app.is_running():
                try:
                    if not self.sim.is_playing():
                        self.sim.render()
                        continue
                    if self.q_hold is not None:
                        self.hold()
                    self.scene.write_data_to_sim()
                    self.sim.step(render=False)
                    n += 1
                    if n % render_interval == 0:
                        self.sim.render()
                    self.scene.update(dt=self.dt)
                except Exception as e:
                    print(f"[DEMO] idle step failed ({e}); still holding the window open", flush=True)
                    self.sim.render()
