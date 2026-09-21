"""MRV Phase 1: AprilTag-based predictive capture of a linearly drifting 3 t MEP.

Runs on `srb/debris_capture_vision`. The controller never reads the ground truth:

    cam_wrist image -> 4 AprilTags -> 16-point PnP -> Cylinder_01 pose (vision)
    -> linear velocity -> Cylinder_01 at t + horizon -> EE target (robot/world frame)
    -> Canadarm3 differential IK (joint position targets) -> existing magnet capture
       (`CaptureManager.attach`, UsdPhysics.FixedJoint) -> holding -> slow retreat

Ground truth (the simulated Cylinder_01 / MEP state) is only used for logging and for
judging the tests.

`mep.motion_mode: six_dof` adds a combined roll/pitch/yaw rate: the prediction becomes
the constant-twist `ConstantTwistPredictor` (orientation integrated on SO(3), Cylinder_01
through a fixed transform), the EE reference and joint feed-forward follow the MEP
angular velocity, and the capture also requires a small relative angular velocity.
`translation_only` (default) keeps the Phase 1 behaviour.

States (Section 26 of the task spec):
    INIT -> SEARCH -> TAG_DETECTED -> POSE_ESTIMATED -> PREDICTING -> APPROACHING
    -> SLOW_APPROACH -> CAPTURE_ATTEMPT -> CAPTURED -> HOLDING -> RETREAT -> SUCCESS
    static scenario: ... PREDICTING -> STATIC_MEASURE -> SUCCESS
    failures: TAG_LOST, POSE_INVALID, PREDICTION_INVALID, APPROACH_TIMEOUT,
              CAPTURE_FAILED, PHYSICS_ERROR (arm holds its joints = safe stop)
"""

import csv
import dataclasses
import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch

from .docking_demo import ArmKinematics, interp_frame
from . import orbit_return, probe_dock
from .frames import Frame, axis_angle, rotation_angle
from .vision import (
    yaw_about_base,
    PROJECT_DIR,
    AprilTagDetector,
    ConstantTwistPredictor,
    Estimate,
    LinearDriftPredictor,
    TagDetection,
    VisionCaptureConfig,
    camera_in_link,
    downsample,
    estimate_constellation_pose,
    intrinsics_from_usd,
    pose_errors,
    project_points,
    rotmat_to_rpy_deg,
    scale_intrinsics,
    so3_exp,
    so3_log,
    tag_centres,
)

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from .vision_task import VisionCaptureTask


class State(Enum):
    INIT = "INIT"
    SEARCH = "SEARCH"
    TAG_DETECTED = "TAG_DETECTED"
    POSE_ESTIMATED = "POSE_ESTIMATED"
    PREDICTING = "PREDICTING"
    APPROACHING = "APPROACHING"
    SLOW_APPROACH = "SLOW_APPROACH"
    CAPTURE_ATTEMPT = "CAPTURE_ATTEMPT"
    CAPTURED = "CAPTURED"
    HOLDING = "HOLDING"
    RETREAT = "RETREAT"
    SUCCESS = "SUCCESS"
    STATIC_MEASURE = "STATIC_MEASURE"
    # failures
    TAG_LOST = "TAG_LOST"
    POSE_INVALID = "POSE_INVALID"
    PREDICTION_INVALID = "PREDICTION_INVALID"
    APPROACH_TIMEOUT = "APPROACH_TIMEOUT"
    CAPTURE_FAILED = "CAPTURE_FAILED"
    PHYSICS_ERROR = "PHYSICS_ERROR"
    ABORTED = "ABORTED"  # ROS cmd/abort
    ## Phase 2 -- Ares1 probe -> satellite thruster docking (`probe_dock.py`).
    ## Enabled by `docking.enabled` (`vision_capture.py --dock`); the capture states
    ## above are unchanged and hand over after HOLDING / RETREAT.
    DOCK_TARGET_ACQUIRE = "DOCK_TARGET_ACQUIRE"
    PRE_DOCK_APPROACH = "PRE_DOCK_APPROACH"
    XY_ALIGN = "XY_ALIGN"
    ORIENTATION_ALIGN = "ORIENTATION_ALIGN"
    ALIGNMENT_CHECK = "ALIGNMENT_CHECK"
    Z_APPROACH = "Z_APPROACH"
    FINAL_INSERTION = "FINAL_INSERTION"
    DOCK_READY = "DOCK_READY"
    DOCKED = "DOCKED"
    DOCK_HOLDING = "DOCK_HOLDING"
    DOCK_FAILED = "DOCK_FAILED"
    ## Phase 3 -- the docked MEP + Client are carried onto the Client reference orbit
    ## (`orbit_return.py`, `--orbit-return`): DOCKED -> ORBIT_TARGET_ACQUIRE -> ORBIT_TRANSFER
    ## -> ORBIT_ARRIVAL_CHECK -> ORBIT_HOLDING -> SUCCESS
    ORBIT_TARGET_ACQUIRE = "ORBIT_TARGET_ACQUIRE"
    ORBIT_TRANSFER = "ORBIT_TRANSFER"
    ORBIT_ARRIVAL_CHECK = "ORBIT_ARRIVAL_CHECK"
    ORBIT_HOLDING = "ORBIT_HOLDING"
    ORBIT_FAILED = "ORBIT_FAILED"


FAILURES = {State.TAG_LOST, State.POSE_INVALID, State.PREDICTION_INVALID, State.APPROACH_TIMEOUT, State.CAPTURE_FAILED, State.PHYSICS_ERROR,
            State.ABORTED, State.DOCK_FAILED, State.ORBIT_FAILED}
# States of the orbit-return phase (the probe tip stays the controlled frame; MEP + Client are joined)
ORBIT_STATES = {State.ORBIT_TARGET_ACQUIRE, State.ORBIT_TRANSFER, State.ORBIT_ARRIVAL_CHECK, State.ORBIT_HOLDING}
# States of the docking phase (the probe tip is the controlled frame in all of them)
DOCKING_STATES = {State.DOCK_TARGET_ACQUIRE, State.PRE_DOCK_APPROACH, State.XY_ALIGN, State.ORIENTATION_ALIGN,
                  State.ALIGNMENT_CHECK, State.Z_APPROACH, State.FINAL_INSERTION, State.DOCK_READY,
                  State.DOCKED, State.DOCK_HOLDING} | ORBIT_STATES
# ... of which these actively command a motion towards the docking axis
DOCKING_MOTION = {State.PRE_DOCK_APPROACH, State.XY_ALIGN, State.ORIENTATION_ALIGN, State.Z_APPROACH, State.FINAL_INSERTION}
TERMINAL = FAILURES | {State.SUCCESS}
TRACKING = {State.APPROACHING, State.SLOW_APPROACH, State.CAPTURE_ATTEMPT, State.STATIC_MEASURE}

# EE contact frame relative to the tag frame when facing it: half turn about X
FLIP = Frame(np.zeros(3), np.diag([1.0, -1.0, -1.0]))

CSV_COLUMNS = [
    # required (task spec Section 25)
    "timestamp",
    "gt_x", "gt_y", "gt_z", "gt_roll", "gt_pitch", "gt_yaw",
    "est_x", "est_y", "est_z", "est_roll", "est_pitch", "est_yaw",
    "position_error_mm", "angle_error_deg",
    "pred_x", "pred_y", "pred_z",
    "ee_x", "ee_y", "ee_z",
    "ee_target_x", "ee_target_y", "ee_target_z",
    "ee_target_distance_m",
    "relative_velocity_mps",
    "mep_linear_velocity_x", "mep_linear_velocity_y", "mep_linear_velocity_z",
    "mep_angular_velocity_x", "mep_angular_velocity_y", "mep_angular_velocity_z",
    "num_detected_tags",
    "capture_state", "capture_success",
    "total_elapsed_time",
    # extra
    "scenario", "est_age_s", "reproj_rms_px", "pnp_inliers", "raw_position_error_mm", "raw_angle_error_deg",
    "est_base_x", "est_base_y", "est_base_z",
    "gt_mep_linear_velocity_x", "gt_mep_linear_velocity_y", "gt_mep_linear_velocity_z",
    "gt_mep_angular_velocity_deg_s", "gt_relative_velocity_mps",
    "ee_to_cylinder_distance_m", "ee_lateral_error_m", "ee_gap_m", "ee_normal_angle_deg",
    "standoff_cmd_m", "fixed_joint_valid", "mep_ee_relative_drift_mm", "mep_ee_relative_drift_deg",
    # 6-DoF extension (appended so the Phase 1 columns keep their order).
    # Quaternions are (w, x, y, z), world frame; angular velocities rad/s, world frame.
    "motion_mode", "prediction_horizon_s",
    "gt_qw", "gt_qx", "gt_qy", "gt_qz",
    "est_qw", "est_qx", "est_qy", "est_qz",
    "pred_qw", "pred_qx", "pred_qy", "pred_qz",
    "ee_qw", "ee_qx", "ee_qy", "ee_qz",
    "ee_target_qw", "ee_target_qx", "ee_target_qy", "ee_target_qz",
    "gt_mep_angular_velocity_x", "gt_mep_angular_velocity_y", "gt_mep_angular_velocity_z",
    "ee_angular_velocity_x", "ee_angular_velocity_y", "ee_angular_velocity_z",
    # capture-condition quantities w.r.t. the vision estimate (the gt_* / ee_* ones above are GT)
    "est_ee_to_cylinder_distance_m", "est_ee_lateral_error_m", "est_ee_normal_angle_deg",
    "est_ee_orientation_error_deg", "est_relative_angular_velocity_rad_s",
    "gt_ee_orientation_error_deg", "gt_relative_angular_velocity_rad_s",
    "pnp_valid", "tracking_valid",
]


ORBIT_CSV_COLUMNS = [
    "run_id", "timestamp", "state",
    "client_ref_x", "client_ref_y", "client_ref_z",
    "orbit_error_m", "orbit_out_of_plane_m", "orbit_nearest_x", "orbit_nearest_y", "orbit_nearest_z",
    "target_x", "target_y", "target_z",
    "client_vx", "client_vy", "client_vz", "client_wx", "client_wy", "client_wz",
    "mep_vx", "mep_vy", "mep_vz", "mep_wx", "mep_wy", "mep_wz",
    "mep_client_drift_mm", "mep_client_drift_deg", "docking_joint_valid", "capture_joint_valid", "arm_joint_margin_rad",
]


@dataclass
class VisionFrame:
    """Result of processing one `cam_wrist` image."""

    t: float
    detections: Dict[int, TagDetection]
    image: Optional[np.ndarray] = None
    cam_w: Optional[Frame] = None
    t_w_y: Optional[Frame] = None  # estimated Cylinder_01 (world)
    reproj_rms: float = math.nan
    inliers: int = 0
    valid: bool = False
    reason: str = ""


@dataclass
class Results:
    scenario: str
    checks: Dict[str, dict] = field(default_factory=dict)
    metrics: Dict[str, object] = field(default_factory=dict)
    failure: Optional[str] = None
    final_state: str = ""

    def check(self, name: str, ok: bool, detail: str = "", **values) -> bool:
        self.checks[name] = {"pass": bool(ok), "detail": detail, **values}
        print(f"[CHECK] {name}: {'PASS' if ok else 'FAIL'}{'  (' + detail + ')' if detail else ''}", flush=True)
        return ok


class Visualizer:
    """3D debug draw (`isaacsim.util.debug_draw`).

    Debug draw is rendered into every render product, including `cam_wrist`, where it
    would hide the tags. It is therefore cleared right before each render whose image
    is used for detection and redrawn right after.
    """

    COLORS = {
        "tag": (0.1, 0.5, 1.0, 1.0),
        "gt": (0.1, 1.0, 0.2, 1.0),
        "est": (1.0, 0.55, 0.0, 1.0),
        "pred": (1.0, 0.1, 1.0, 1.0),
        "ee": (0.0, 1.0, 1.0, 1.0),
        "target": (1.0, 1.0, 1.0, 1.0),
        "velocity": (1.0, 1.0, 0.0, 1.0),
    }
    # Linear velocity arrow = v * this [s] (the real 0.3 s displacement is only mm long)
    VELOCITY_ARROW_SEC = 10.0

    def __init__(self, enabled: bool):
        self.draw = None
        self.num_drawn = 0
        if not enabled:
            return
        try:
            from isaacsim.util.debug_draw import _debug_draw

            self.draw = _debug_draw.acquire_debug_draw_interface()
        except Exception as e:  # extension missing -> 2D overlay only
            print(f"[VIS] debug draw unavailable: {e}", flush=True)
        self._items: Tuple[list, list, list, list, list, list, list] = ([], [], [], [], [], [], [])

    @property
    def available(self) -> bool:
        return self.draw is not None

    # Axis triads: X red, Y green, Z blue, scaled by `brightness`
    AXIS_COLORS = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    def set_scene(self, tags_w: List[np.ndarray], centre_w, est_w, gt_w, pred_w, ee_w, target_w,
                  axes: Sequence[Tuple[Frame, float, float, float]] = (),
                  vectors: Sequence[Tuple[np.ndarray, np.ndarray, str, float]] = ()):
        """`axes`: (frame, axis length [m], line width, brightness) triads.
        `vectors`: (start, end, colour name, line width) segments."""
        pts, cols, sizes, a, b, lcol, lw = [], [], [], [], [], [], []

        def point(p, c, s):
            if p is not None:
                pts.append(tuple(float(x) for x in p))
                cols.append(self.COLORS[c] if isinstance(c, str) else c)
                sizes.append(s)

        def line(p, q, c, w):
            if p is not None and q is not None:
                a.append(tuple(float(x) for x in p))
                b.append(tuple(float(x) for x in q))
                lcol.append(self.COLORS[c] if isinstance(c, str) else c)
                lw.append(w)

        for t in tags_w:
            point(t, "tag", 10)
        for i in range(len(tags_w)):
            line(tags_w[i], tags_w[(i + 1) % len(tags_w)], "tag", 2)
        point(centre_w, "tag", 14)
        point(gt_w, "gt", 14)
        point(est_w, "est", 12)
        point(pred_w, "pred", 12)
        point(ee_w, "ee", 14)
        point(target_w, "target", 10)
        line(ee_w, target_w, "target", 1)
        for frame, length, width, bright in axes:
            if frame is None:
                continue
            for i, rgb in enumerate(self.AXIS_COLORS):
                line(frame.pos, frame.pos + length * frame.rot[:, i], (*(bright * c for c in rgb), 1.0), width)
        for p, q, c, width in vectors:
            line(p, q, c, width)
        self._items = (pts, cols, sizes, a, b, lcol, lw)

    def clear(self):
        if self.draw is not None:
            self.draw.clear_points()
            self.draw.clear_lines()

    def show(self):
        if self.draw is None:
            return
        self.clear()
        pts, cols, sizes, a, b, lcol, lw = self._items
        if pts:
            self.draw.draw_points(pts, cols, sizes)
        if a:
            self.draw.draw_lines(a, b, lcol, lw)
        self.num_drawn = max(self.num_drawn, len(pts) + len(a))


class VisionCaptureDemo:
    def __init__(self, env, sim_app: "SimulationApp", scenario: str, headless: bool, out_dir: Path, csv_path: Path, label: Optional[str] = None):
        assert scenario in ("static", "dynamic")
        self.env = env
        self.task: "VisionCaptureTask" = env.unwrapped
        t = self.task
        self.cfg: VisionCaptureConfig = t.vision_cfg
        self.scenario = scenario
        self.label = label or scenario  # run name (output file names, CSV `scenario` column)
        self.sim_app = sim_app
        self.headless = headless
        self.sim = t.sim
        self.scene = t.scene
        self.stage = t.scene.stage
        self.geo = t.docking_geometry
        self.capture = t._capture
        self.mep = t._obj
        self.camera = t.scene[self.cfg.camera.name]
        self.contacts = self.scene.sensors.get("contacts_robot")
        a = self.cfg.approach
        self.arm = ArmKinematics(t, self.geo.ee_contact_link, a.ik_lambda)
        self.dt = self.sim.get_physics_dt()
        self.render_interval = max(1, t.cfg.sim.render_interval)

        ## Fixed transforms
        self.t_l_c = camera_in_link(self.cfg.camera)  # link -> camera (mount)
        self.t_t_y = t.t_t_y  # tag constellation -> Cylinder_01 (design time)
        self.t_y_t = t.t_t_y.inv()
        self.t_m_y = t.t_m_y  # MEP body -> Cylinder_01 (ground truth, logging only)
        self.t_m_t = t.t_m_t
        self.face_offset = self.cfg.apriltag.face_offset_m

        ## Intrinsics (from the Isaac Sim camera) + cross-check against the USD prim.
        ## With supersampling the camera renders `ss` x larger; the processed image is
        ## area-downsampled to the configured resolution and K scaled accordingly.
        self.ss = int(self.cfg.camera.supersample)
        self.k_render = self.camera.data.intrinsic_matrices[0].cpu().numpy().astype(float)
        self.k = scale_intrinsics(self.k_render, self.ss)
        prim = self.stage.GetPrimAtPath(f"{self.geo.link_path}/{self.cfg.camera.name}")
        from pxr import UsdGeom

        cam = UsdGeom.Camera(prim)
        self.usd_camera = {
            "prim": str(prim.GetPath()),
            "focal_length_mm": float(cam.GetFocalLengthAttr().Get()),
            "horizontal_aperture_mm": float(cam.GetHorizontalApertureAttr().Get()),
            "vertical_aperture_mm": float(cam.GetVerticalApertureAttr().Get()),
        }
        self.k_usd = intrinsics_from_usd(self.usd_camera["focal_length_mm"], self.usd_camera["horizontal_aperture_mm"], self.cfg.camera.width, self.cfg.camera.height)
        self.k_usd_render = intrinsics_from_usd(self.usd_camera["focal_length_mm"], self.usd_camera["horizontal_aperture_mm"], self.cfg.camera.width * self.ss, self.cfg.camera.height * self.ss)

        self.detector = AprilTagDetector(self.cfg.apriltag)
        ## translation_only: Phase 1 predictor (unchanged). six_dof: constant twist of a
        ## reference frame R rigid on the MEP, mapped to Cylinder_01 with a fixed
        ## transform: T_W_Y(t+h) = T_W_R(t+h) @ T_R_Y
        ##   tag:     R = tag constellation T (the PnP-measured frame), T_R_Y = T_T_Y
        ##   mep_com: R = MEP centre of mass with the MEP body axes, T_R_Y = T_COM_M @ T_M_T @ T_T_Y.
        ##            The COM offset in the MEP body is a mass property of the model, read
        ##            once here (like T_T_Y is measured once from the USD) -- never the live pose.
        self.six_dof = self.cfg.mep.six_dof
        root_rot = Frame.from_pos_quat(np.zeros(3), self.mep.data.root_quat_w[0].tolist()).rot
        self.com_in_mep = root_rot.T @ (self.mep.data.root_com_pos_w[0] - self.mep.data.root_pos_w[0]).cpu().numpy()
        if self.cfg.prediction.reference_frame == "mep_com":
            self.t_ref_y = Frame(self.com_in_mep, np.eye(3)).inv() @ self.t_m_t @ self.t_t_y
        else:
            self.t_ref_y = self.t_t_y
        if self.six_dof:
            self.predictor = ConstantTwistPredictor(self.cfg.prediction, self.t_ref_y)
        else:
            self.predictor = LinearDriftPredictor(self.cfg.prediction)
        self.vis = Visualizer(self.cfg.logging.debug_draw)
        ## Optional ROS 2 interface (telemetry out, start / abort / capture_enable in)
        self.ros = None
        self._next_ros_t = 0.0
        self._start_wait_logged = False
        self._capture_wait_logged = False
        if self.cfg.ros.enabled:
            from .ros_interface import VisionRosInterface

            self.ros = VisionRosInterface(self.cfg.ros, (self.cfg.camera.width, self.cfg.camera.height), self.k, self.cfg.camera.name)

        ## State
        self.state = State.INIT
        self.state_time = 0.0
        self.sim_time = 0.0
        self.wall_start = time.time()
        self.done = False
        self.results = Results(scenario)
        self.ref: Optional[Frame] = None  # EE contact reference pose (rate limited)
        self.goal: Optional[Frame] = None  # EE contact target pose (from prediction)
        self.standoff: float = a.approach_standoff_m
        self.q_hold: Optional[torch.Tensor] = None
        self.last_vision: Optional[VisionFrame] = None
        self.last_valid: Optional[VisionFrame] = None
        self._next_vision_t = 0.0
        self._next_log_t = 0.0
        self._next_overlay_t = 0.0
        self._next_status_t = 0.0
        self._settled_since: Optional[float] = None
        self._capture_range_logged = False
        self._capture_info: Dict[str, float] = {}
        self._rel_at_capture: Optional[Frame] = None
        self._hold: Dict[str, float] = {}
        self._static_idx = 0
        self._static_samples: List[Tuple[float, float, float]] = []
        self._static_stats: List[dict] = []
        self._retreat: Optional[dict] = None
        self._max_contact = 0.0
        self._ambiguous = 0
        self._cylinder_color = None
        self.rows: List[dict] = []
        ## Phase 2 (docking): bookkeeping, probe camera handles, CSV
        self._dock: Dict[str, object] = {}
        self._dock_ref: Optional[Frame] = None
        self._dock_align_since: Optional[float] = None
        self._dock_axial_hold: Optional[float] = None
        self._dock_started: Optional[float] = None
        self._dock_speed = 0.0
        self._calib_rejected = False
        self._depth_calibrated_t: Optional[float] = None
        self._tip_history: List[Tuple[float, np.ndarray]] = []
        self._depth_offset: Optional[float] = None
        self._depth_reason: str = "not read yet"
        self._dock_rows: List[dict] = []
        self._dock_csv = self._dock_csv_file = None
        self._probe_frames: List[Path] = []
        self._next_dock_log_t = 0.0
        self._next_probe_rgb_t = 0.0
        self._probe_cam = None
        self._probe_k = np.eye(3)
        self._cam_to_tip = 0.0
        if self.cfg.docking.enabled and self.cfg.probe_camera.enabled:
            self._probe_cam = t.scene[self.cfg.probe_camera.name]
            self._probe_k = self._probe_cam.data.intrinsic_matrices[0].cpu().numpy().astype(float)
            # Negative: the camera sits in *front* of the tip (see `probe_dock.depth_to_dock_distance`)
            self._cam_to_tip = -float(self.cfg.probe_camera.offset_from_tip_m)
        ## Client reference orbit (`orbit_reference.enabled`): the scene part is built by the task
        self.orbit = getattr(t, "orbit", None)
        self.orbit_cfg = self.cfg.orbit_reference
        self._observe_only = bool(self.orbit is not None and self.orbit_cfg.observe_only)
        self._orbit: Dict[str, object] = {}
        self._orbit_goal: Optional[Frame] = None
        self._orbit_tangent: Optional[np.ndarray] = None
        self._orbit_view_saved = False
        self._free_flight: Dict[str, dict] = {}
        self._orbit_rows: List[dict] = []
        self._orbit_csv = self._orbit_csv_file = None
        # six_dof: GT pose at t = 0 for the runtime angular-velocity frame check
        self._w_frame_ref: Optional[Tuple[float, Frame]] = None
        self._w_frame_result: Optional[dict] = None

        ## Output
        self.out_dir = out_dir
        self.overlay_dir = out_dir / f"{self.label}_overlay"
        self.overlay_dir.mkdir(parents=True, exist_ok=True)
        for old in self.overlay_dir.glob("*.png"):  # overlays of a previous run
            old.unlink()
        self.probe_dir_name = f"{self.label}_probe_rgb"
        self._probe_dir = out_dir / self.probe_dir_name
        if self.cfg.docking.enabled:
            self._probe_dir.mkdir(parents=True, exist_ok=True)
            for old in self._probe_dir.glob("*.png"):
                old.unlink()
            self.dock_csv_path = out_dir / f"{self.label}_docking.csv"
            if self.cfg.logging.csv_enabled:
                self._dock_csv_file = open(self.dock_csv_path, "w", newline="")
                self._dock_csv = csv.DictWriter(self._dock_csv_file, fieldnames=list(probe_dock.CSV_COLUMNS))
                self._dock_csv.writeheader()
        if self.orbit is not None and self.cfg.logging.csv_enabled:
            self.orbit_csv_path = out_dir / f"{self.label}_orbit.csv"
            self._orbit_csv_file = open(self.orbit_csv_path, "w", newline="")
            self._orbit_csv = csv.DictWriter(self._orbit_csv_file, fieldnames=ORBIT_CSV_COLUMNS)
            self._orbit_csv.writeheader()
        self.csv_path = csv_path
        self._csv_file = self._csv = None
        if self.cfg.logging.csv_enabled:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = open(csv_path, "w", newline="")
            self._csv = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS)
            self._csv.writeheader()
        self._overlays: List[Path] = []

    ##############
    ### Frames ###
    ##############

    def mep_frame(self) -> Frame:
        return Frame.from_pos_quat(self.mep.data.root_pos_w[0].tolist(), self.mep.data.root_quat_w[0].tolist())

    def gt_cylinder(self) -> Frame:
        """Ground truth Cylinder_01 pose (logging / test judgement only)."""
        return self.mep_frame() @ self.t_m_y

    def ee_pose(self) -> Frame:
        return self.arm.link_pose() @ self.geo.ee_contact_link

    def cam_pose(self) -> Frame:
        return self.arm.link_pose() @ self.t_l_c

    def base_frame(self) -> Frame:
        return self.arm.base

    def ee_velocity(self) -> np.ndarray:
        """Linear velocity of the EE contact point (proprioception)."""
        lid = self.arm.link_id
        v = self.task._robot.data.body_lin_vel_w[0, lid].cpu().numpy()
        w = self.task._robot.data.body_ang_vel_w[0, lid].cpu().numpy()
        r = self.arm.link_pose().rot @ self.geo.ee_contact_link.pos
        return v + np.cross(w, r)

    def ee_angular_velocity(self) -> np.ndarray:
        """Angular velocity [rad/s] (world) of the EE (= last link, rigid tool)."""
        return self.task._robot.data.body_ang_vel_w[0, self.arm.link_id].cpu().numpy()

    def est_mep_twist_at(self, p: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Vision-estimated MEP velocity at world point `p` and angular velocity (world).

        translation_only: (v, None) -- exactly the Phase 1 velocity. six_dof: the rigid
        velocity field v + w x (p - p_ref) and w."""
        v, w = self.predictor.twist()
        if v is None or w is None:
            return v, w
        return self.predictor.velocity_at(p, self.sim_time), w

    def feedforward(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Feed-forward (v, w) for the EE reference: MEP motion at the EE contact point."""
        return self.est_mep_twist_at(self.ee_pose().pos)

    def gt_mep_velocity_at(self, p: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        d = self.mep.data
        v = d.root_com_lin_vel_w[0].cpu().numpy()
        w = d.root_com_ang_vel_w[0].cpu().numpy()
        c = d.root_com_pos_w[0].cpu().numpy()
        return v + np.cross(w, p - c), w

    def ee_goal(self, t_w_t: Frame, standoff: float) -> Frame:
        """EE contact pose facing the tag frame, `standoff` from the MEP face."""
        return t_w_t @ Frame(np.array([0.0, 0.0, standoff - self.face_offset]), np.eye(3)) @ FLIP

    ##############
    ### Vision ###
    ##############

    def estimate_now(self) -> Optional[Frame]:
        """System estimate of Cylinder_01 now: fitted position, window-mean orientation."""
        if self.last_valid is None:
            return None
        return self.predictor.estimate(self.sim_time)

    def predicted(self) -> Optional[Frame]:
        return self.predictor.predict(self.sim_time)

    def estimate_age(self) -> float:
        return math.inf if self.last_valid is None else self.sim_time - self.last_valid.t

    def process_image(self):
        """Detect + PnP on the image rendered at the current physics state."""
        rgb = self.camera.data.output["rgb"][0].cpu().numpy()
        img = np.ascontiguousarray(downsample(rgb[..., :3], self.ss))
        dets = self.detector.detect(img)
        vf = VisionFrame(self.sim_time, dets, image=img, cam_w=self.cam_pose())
        # Prior for the planar ambiguity: fused orientation of the constellation seen
        # from the current camera pose
        prior = None
        # (only once the window holds enough images, so a single early wrong solution
        # cannot become its own prior)
        # (six_dof: the fitted orientation propagated to this image's time)
        fused = self.predictor.orientation_at(vf.t) if len(self.predictor.history) >= self.cfg.prediction.min_samples else None
        if fused is not None:
            prior = vf.cam_w.rot.T @ fused @ self.t_y_t.rot
        pose = estimate_constellation_pose(dets, self.cfg.apriltag, self.k, prior_rot_c=prior)
        if pose is not None and pose.ambiguous:
            self._ambiguous += 1
        if pose is None:
            vf.reason = f"{len(dets)}/4 tags" if len(dets) < self.cfg.apriltag.min_tags_for_pose else "PnP failed"
        elif pose.reproj_rms_px > self.cfg.apriltag.max_reprojection_rms_px:
            vf.reason = f"reprojection RMS {pose.reproj_rms_px:.2f} px"
            vf.reproj_rms = pose.reproj_rms_px
        else:
            vf.t_w_y = vf.cam_w @ pose.t_c_t @ self.t_t_y
            vf.reproj_rms = pose.reproj_rms_px
            vf.inliers = pose.num_inliers
            vf.valid = True
            self.predictor.add(Estimate(vf.t, vf.t_w_y))
            self.last_valid = vf
        self.last_vision = vf
        if self.sim_time >= self._next_overlay_t:
            self._next_overlay_t = self.sim_time + self.cfg.logging.overlay_every_sec
            self.save_overlay(vf)

    ###############
    ### Control ###
    ###############

    def track(self, goal: Frame, speed: float, v_ff: Optional[np.ndarray], max_step: Optional[float] = None,
              w_ff: Optional[np.ndarray] = None, lag_limit: float = 0.05):
        """Move the reference towards `goal` (rate limited + feed-forward), then IK.

        `v_ff` [m/s] / `w_ff` [rad/s] (world): MEP motion the reference follows every
        step; `w_ff` is only given in six_dof (rotating target)."""
        a = self.cfg.approach
        if self.ref is None:
            self.ref = self.ee_pose()
        ee = self.ee_pose()
        lag_p, lag_a = pose_errors(ee, self.ref)
        pos = self.ref.pos.copy()
        if v_ff is not None:
            pos = pos + v_ff * self.dt
        rot = self.ref.rot
        if w_ff is not None:
            rot = so3_exp(w_ff * self.dt) @ rot  # world-frame w: left multiplication
        # The reference only advances while the arm keeps up with it
        if lag_p < lag_limit and lag_a < 3.0:
            d = goal.pos - pos
            n = float(np.linalg.norm(d))
            if n > 1e-9:
                pos = pos + d / n * min(n, speed * self.dt)
            ang = rotation_angle(rot, goal.rot)
            if ang > 1e-9:
                s = min(1.0, math.radians(a.speed_ang_deg_s) * self.dt / ang)
                rot = interp_frame(Frame(pos, rot), Frame(pos, goal.rot), s).rot
        self.ref = Frame(pos, rot)
        q = self.arm.ik_joint_target(self.ref, a.max_joint_step_rad if max_step is None else max_step)
        self.task._robot.set_joint_position_target(q, joint_ids=self.arm.joint_ids)
        self.set_velocity_feedforward(v_ff, w_ff)

    def set_velocity_feedforward(self, v_ff: Optional[np.ndarray], w_ff: Optional[np.ndarray] = None):
        """Joint velocity targets that move the EE with the estimated MEP velocity.

        The implicit joint drives are PD on position *and* velocity. With position
        targets only, a target moving at v is trailed by v * damping / stiffness
        (0.625 s for the Canadarm3); q_dot* = J^+ [v; w] removes that lag (w = 0 unless
        six_dof gives the MEP angular velocity).
        """
        n = len(self.arm.joint_ids)
        if v_ff is None:
            qd = torch.zeros((1, n), device=self.arm.device)
        else:
            j = self.arm.jacobian_base()[0]  # (6, n), base frame, EE contact point
            twist = torch.zeros(6, dtype=j.dtype, device=j.device)
            twist[:3] = torch.tensor(self.arm.base.rot.T @ v_ff, dtype=j.dtype, device=j.device)
            if w_ff is not None:
                twist[3:] = torch.tensor(self.arm.base.rot.T @ w_ff, dtype=j.dtype, device=j.device)
            lam = self.cfg.approach.ik_lambda
            jjt = j @ j.T + (lam ** 2) * torch.eye(6, dtype=j.dtype, device=j.device)
            qd = (j.T @ torch.linalg.solve(jjt, twist)).unsqueeze(0)
        self.task._robot.set_joint_velocity_target(qd, joint_ids=self.arm.joint_ids)

    def hold(self):
        if self.q_hold is None:
            self.q_hold = self.arm.joint_pos().clone()
        self.task._robot.set_joint_position_target(self.q_hold, joint_ids=self.arm.joint_ids)
        self.set_velocity_feedforward(None)

    def goto(self, state: State, reason: str = ""):
        print(f"[STATE] {self.state.value} -> {state.value}  (t={self.sim_time:.2f} s){'  ' + reason if reason else ''}", flush=True)
        if state in FAILURES:
            self.results.failure = f"{state.value}: {reason}"
            print(f"[FAIL] {state.value}: {reason}", flush=True)
            if self.orbit is not None:
                self.print_orbit_failure_context()
            self.q_hold = None  # safe stop: hold the joints where they are
        self.state = state
        self.state_time = 0.0
        self._settled_since = None

    ##################
    ### Conditions ###
    ##################

    def capture_metrics(self, y: Optional[Frame], v_mep: Optional[np.ndarray], w_mep: Optional[np.ndarray] = None) -> Dict[str, float]:
        """Capture conditions w.r.t. a Cylinder_01 pose `y` (estimated or GT).

        `v_mep`: MEP velocity at the EE contact point [m/s]; `w_mep`: MEP angular
        velocity [rad/s] (world), only needed for `rel_ang_vel`."""
        ee = self.ee_pose()
        if y is None:
            return {k: math.nan for k in ("distance", "gap", "lateral", "angle", "orientation", "rel_vel", "rel_ang_vel")}
        t = y @ self.t_y_t  # tag frame (face normal = +Z)
        n = t.rot[:, 2]
        rel = ee.pos - t.pos
        gap_t = float(rel @ n)
        return {
            "distance": float(np.linalg.norm(ee.pos - y.pos)),
            "gap": gap_t + self.face_offset,  # EE face above the MEP face
            "lateral": float(np.linalg.norm(rel - gap_t * n)),
            # tilt of the EE axis w.r.t. the face normal (the capture condition)
            "angle": math.degrees(axis_angle(ee.rot[:, 2], -n)),
            # full 3-axis orientation error to the EE capture orientation (incl. roll about
            # the axis; logged, the rotationally symmetric magnet does not need it)
            "orientation": math.degrees(rotation_angle(ee.rot, (t @ FLIP).rot)),
            "rel_vel": float(np.linalg.norm(self.ee_velocity() - v_mep)) if v_mep is not None else math.nan,
            "rel_ang_vel": float(np.linalg.norm(self.ee_angular_velocity() - w_mep)) if w_mep is not None else math.nan,
        }

    def est_capture_metrics(self) -> Dict[str, float]:
        """Capture metrics w.r.t. the vision estimate (what the controller uses)."""
        v, w = self.est_mep_twist_at(self.ee_pose().pos)
        return self.capture_metrics(self.estimate_now(), v, w)

    def gt_capture_metrics(self) -> Dict[str, float]:
        """Capture metrics w.r.t. the ground truth (logging / judgement only)."""
        v_gt, w_gt = self.gt_mep_velocity_at(self.ee_pose().pos)
        return self.capture_metrics(self.gt_cylinder(), v_gt, w_gt)

    def tracking_ok(self) -> Tuple[bool, str]:
        lv = self.last_vision
        if self.estimate_age() > self.cfg.vision.tag_loss_timeout_s:
            return False, f"no valid pose for {self.estimate_age():.2f} s"
        if lv is None or not lv.valid:
            return False, f"latest image invalid ({lv.reason if lv else 'none'})"
        if self.predictor.velocity() is None:
            return False, "no velocity estimate"
        return True, "ok"

    def capture_conditions(self) -> Tuple[bool, Dict[str, float], List[str]]:
        c = self.cfg.capture
        # translation_only: MEP velocity v (as in Phase 1); six_dof: v + w x r at the EE
        m = self.est_capture_metrics()
        failed = []
        if not m["distance"] <= c.max_distance_m:
            failed.append(f"distance {m['distance']:.3f} m > {c.max_distance_m}")
        if not m["angle"] <= c.max_angle_deg:
            failed.append(f"angle {m['angle']:.2f} deg > {c.max_angle_deg}")
        if not m["rel_vel"] <= c.max_relative_velocity_mps:
            failed.append(f"relative velocity {m['rel_vel']:.4f} m/s > {c.max_relative_velocity_mps}")
        if self.six_dof and not m["rel_ang_vel"] <= c.max_relative_angular_velocity_rad_s:
            failed.append(f"relative angular velocity {m['rel_ang_vel']:.4f} rad/s > {c.max_relative_angular_velocity_rad_s}")
        if not (m["gap"] > 0.0 and m["lateral"] <= c.max_lateral_m):
            failed.append(f"approach direction (gap {m['gap']*1000:.1f} mm, lateral {m['lateral']*1000:.1f} mm)")
        ok, why = self.tracking_ok()
        if not ok:
            failed.append(f"tracking: {why}")
        return not failed, m, failed

    def check_physics(self) -> Optional[str]:
        d = self.mep.data
        vals = [d.root_pos_w, d.root_quat_w, d.root_com_lin_vel_w, d.root_com_ang_vel_w, self.task._robot.data.joint_pos, self.task._robot.data.joint_vel]
        if not all(bool(torch.isfinite(x).all()) for x in vals):
            return "NaN/Inf in the MEP or robot state"
        v = float(torch.norm(d.root_com_lin_vel_w[0]))
        w = math.degrees(float(torch.norm(d.root_com_ang_vel_w[0])))
        if v > 0.5:
            return f"MEP speed {v:.3f} m/s (explosion)"
        # Phase 1 limit 5 deg/s on top of the commanded rate (0 in translation_only)
        w_limit = 5.0 + math.degrees(float(np.linalg.norm(self.cfg.mep.angular_velocity_w())))
        if w > w_limit:
            return f"MEP angular rate {w:.2f} deg/s > {w_limit:.2f} (unintended rotation)"
        qd = float(torch.abs(self.task._robot.data.joint_vel[0]).max())
        if qd > 1.0:
            return f"robot joint speed {qd:.2f} rad/s (joint explosion)"
        if (self.state in (State.CAPTURED, State.HOLDING, State.RETREAT) or self.state in DOCKING_STATES) and not self.capture.is_attached(0):
            return "FixedJoint missing after capture"
        if self.contacts is not None and not self.capture.is_attached(0):
            f = float(torch.norm(self.contacts.data.net_forces_w[0], dim=-1).max())
            self._max_contact = max(self._max_contact, f)
            if f > 200.0:
                return f"unexpected contact on the arm: {f:.0f} N"
        return self.check_free_flight()

    ###############
    ### Stepping ###
    ###############

    def start(self):
        """t = 0: report the setup and (optionally) place the arm at the observation pose."""
        c = self.cfg
        r = self.results
        t = self.task
        print("[INIT] ---- MRV Phase 1 vision capture ----", flush=True)
        print(f"[INIT] scenario {self.scenario}, motion mode {c.mep.motion_mode}, MEP mass {float(self.mep.root_physx_view.get_masses().sum()):.1f} kg, gravity {tuple(t.cfg.sim.gravity)}", flush=True)
        v0 = self.mep.data.root_com_lin_vel_w[0].cpu().numpy()
        w0 = self.mep.data.root_com_ang_vel_w[0].cpu().numpy()
        v_cmd, w_cmd = c.mep.linear_velocity_w(), c.mep.angular_velocity_w()
        print(f"[INIT] MEP initial linear velocity {np.round(v0, 4).tolist()} m/s (|v| {np.linalg.norm(v0):.4f}), angular {np.round(w0, 5).tolist()} rad/s "
              f"= {np.round(np.degrees(w0), 4).tolist()} deg/s (world)", flush=True)
        if not self.six_dof and np.linalg.norm(c.mep.angular_velocity_rad_s) > 0.0:
            print(f"[INIT] translation_only: mep.angular_velocity_rad_s {c.mep.angular_velocity_rad_s} is not applied", flush=True)
        # MEP centre of mass relative to the root origin (world): the point the linear
        # velocity refers to, and the lever arm of the rotation
        com_offset = (self.mep.data.root_com_pos_w[0] - self.mep.data.root_pos_w[0]).cpu().numpy()
        self._w_frame_ref = (self.sim_time, self.mep_frame())
        print(f"[INIT] K (cam_wrist) = {np.round(self.k, 3).tolist()}", flush=True)
        print(f"[INIT] USD camera {self.usd_camera}", flush=True)
        print(f"[INIT] T_link_cam pos {self.t_l_c.pos.tolist()} quat(wxyz) {np.round(self.t_l_c.quat, 4).tolist()}", flush=True)
        print(f"[INIT] T_mep_tags pos {np.round(self.t_m_t.pos, 4).tolist()} quat {np.round(self.t_m_t.quat, 4).tolist()}", flush=True)
        print(f"[INIT] T_tags_cylinder01 pos {np.round(self.t_t_y.pos, 5).tolist()} quat {np.round(self.t_t_y.quat, 5).tolist()}", flush=True)
        print(f"[INIT] Cylinder_01 radius {t.cylinder_radius*1000:.1f} mm, height {t.cylinder_height*1000:.1f} mm", flush=True)
        mass = float(self.mep.root_physx_view.get_masses().sum())
        r.metrics["setup"] = {
            "mep_mass_kg": mass,
            "gravity": list(t.cfg.sim.gravity),
            "mep_initial_linear_velocity_mps": v0.tolist(),
            "mep_initial_angular_velocity_deg_s": np.degrees(w0).tolist(),
            "mep_initial_angular_velocity_rad_s": w0.tolist(),
            "motion_mode": c.mep.motion_mode,
            "commanded_linear_velocity_mps": v_cmd.tolist(),
            "commanded_angular_velocity_rad_s_world": w_cmd.tolist(),
            "mep_com_offset_from_root_w_m": com_offset.tolist(),
            "mep_com_in_body_m": self.com_in_mep.tolist(),
            "prediction_reference_frame": c.prediction.reference_frame,
            "t_reference_cylinder01": {"pos": self.t_ref_y.pos.tolist(), "quat_wxyz": list(self.t_ref_y.quat)},
            "prediction_horizon_s": c.prediction.horizon_sec,
            "physics_dt": self.dt,
            "render_interval": self.render_interval,
            "K": self.k.tolist(),
            "K_render": self.k_render.tolist(),
            "supersample": self.ss,
            "K_from_usd_attributes": self.k_usd.tolist(),
            "usd_camera": self.usd_camera,
            "t_link_cam": {"pos": self.t_l_c.pos.tolist(), "quat_wxyz": list(self.t_l_c.quat)},
            "t_mep_tags": {"pos": self.t_m_t.pos.tolist(), "quat_wxyz": list(self.t_m_t.quat)},
            "t_tags_cylinder01": {"pos": self.t_t_y.pos.tolist(), "quat_wxyz": list(self.t_t_y.quat)},
            "cylinder01_radius_m": t.cylinder_radius,
            "cylinder01_height_m": t.cylinder_height,
            "tag_layout": t.layout_detail,
            "ee_contact_in_link": {"pos": self.geo.ee_contact_link.pos.tolist(), "quat_wxyz": list(self.geo.ee_contact_link.quat)},
            "config": c.to_dict(),
        }
        r.check("MEP mass is 3000 kg", abs(mass - c.mep.mass_kg) < 1.0 and abs(c.mep.mass_kg - 3000.0) < 1e-6, f"{mass:.1f} kg")
        r.check("Zero gravity", all(abs(g) < 1e-9 for g in t.cfg.sim.gravity), f"gravity {tuple(t.cfg.sim.gravity)}")
        if not self.six_dof:
            r.check("MEP linear drift, no rotation", float(np.linalg.norm(w0)) < 1e-6 and abs(float(np.linalg.norm(v0)) - c.mep.linear_velocity_mps) < 1e-4,
                    f"|v| {np.linalg.norm(v0):.4f} m/s, |w| {np.degrees(np.linalg.norm(w0)):.4f} deg/s")
        else:
            # Isaac Lab writes init_state lin_vel / ang_vel as the root COM velocity in
            # the world frame; reading back the configured vectors confirms that at runtime
            r.check("MEP 6-DoF initial velocity (linear + angular, world)",
                    float(np.linalg.norm(v0 - v_cmd)) < 1e-4 and float(np.linalg.norm(w0 - w_cmd)) < 1e-5,
                    f"v {np.round(v0, 5).tolist()} (cmd {np.round(v_cmd, 5).tolist()}) m/s, w {np.round(w0, 6).tolist()} (cmd {np.round(w_cmd, 6).tolist()}) rad/s, "
                    f"COM offset from root {np.round(com_offset, 3).tolist()} m")
        r.check("Tag layout clear of Cylinder_01 and inside the face", t.layout_ok, t.layout_detail)
        r_ee = float(t.ee_cylinder_radius)
        r.metrics["setup"]["ee_cylinder_radius_m"] = r_ee
        r.check("EE capture cylinder and Cylinder_01 marker share one radius", abs(r_ee - t.cylinder_radius) < 5e-4 and abs(r_ee - c.capture.cylinder_radius_m) < 5e-4,
                f"measured in the stage: EE cylinder r {r_ee*1000:.1f} mm, Cylinder_01 r {t.cylinder_radius*1000:.1f} mm (config {c.capture.cylinder_radius_m*1000:.0f} mm), "
                f"Cylinder_01 height {t.cylinder_height*1000:.1f} mm, ior {c.capture.cylinder_ior}")
        r.check("Intrinsics consistent with the USD camera", float(np.abs(self.k_render - self.k_usd_render).max()) < 1e-3 and float(np.abs(self.k - self.k_usd).max()) < 1e-3,
                f"max |K - K_usd| {np.abs(self.k - self.k_usd).max():.2e} (render {self.ss}x: {np.abs(self.k_render - self.k_usd_render).max():.2e})")

        a = c.approach
        # Configured search pose: nominal (planned) MEP pose, not the live one
        self.observe_pose = self.ee_goal(t.mep_nominal @ self.t_m_t, a.observe_distance_m)
        if a.start_at_observe_pose:
            ok, q, pe, ae = self.arm.solve_kinematic(self.observe_pose, self.arm.joint_pos().clone())
            r.check("Observation pose reachable (IK)", ok, f"IK err {pe*1000:.3f} mm / {math.degrees(ae):.4f} deg")
            if a.start_yaw_offset_deg != 0.0:
                # Start swung about the base axis: the camera does not see the MEP and
                # SEARCH has to find it (the observation pose stays the configured target)
                start = yaw_about_base(self.observe_pose, self.arm.base, a.start_yaw_offset_deg)
                ok, q, pe, ae = self.arm.solve_kinematic(start, q)
                r.check("Yawed start pose reachable (IK)", ok, f"{a.start_yaw_offset_deg:+.1f} deg about the base axis, IK err {pe*1000:.3f} mm / {math.degrees(ae):.4f} deg")
                print(f"[SEARCH] arm starts swung {a.start_yaw_offset_deg:+.1f} deg in azimuth from the observation pose", flush=True)
            self.task._robot.set_joint_position_target(q, joint_ids=self.arm.joint_ids)
            self.q_hold = q.clone()
        self.ref = self.ee_pose()
        if self.orbit is not None:
            self.report_orbit_setup()
        ## Docking phase reported next to the capture setup
        if c.docking.enabled:
            geo = self.geo
            r.metrics["setup"]["docking"] = {
                "probe_tip_in_mep_m": geo.probe_dock.pos.tolist(),
                "probe_axis_in_mep": geo.probe_dock.rot[:, 2].tolist(),
                "probe_tip_radius_m": geo.probe.tip_radius,
                "sat_dock_in_satellite_m": geo.sat_dock.pos.tolist(),
                "sat_dock_axis_in_satellite": geo.sat_dock.rot[:, 2].tolist(),
                "dock_depth_m": float(t.cfg.docking.dock_depth),
                "backstop_gap_m": float(t.cfg.docking.backstop_gap),
                "nozzle_exit_radius_m": geo.nozzle.exit_radius,
                "nozzle_inner_radius_at_dock_m": geo.nozzle.inner_radius(t.cfg.docking.dock_depth),
                "satellite_velocity_mps": (np.asarray(c.docking.satellite_drift_direction) * c.docking.satellite_velocity_mps).tolist(),
                "probe_camera": ({"pos_in_mep_m": t.probe_cam_in_mep.pos.tolist(),
                                  "axis_in_mep": t.probe_cam_in_mep.rot[:, 2].tolist(),
                                  "K": self._probe_k.tolist(),
                                  "camera_to_tip_m": self._cam_to_tip}
                                 if getattr(t, "probe_cam_in_mep", None) is not None else None),
            }
            sat_v0 = self.task._satellite.data.root_com_lin_vel_w[0].cpu().numpy()
            sat_cmd = np.asarray(c.docking.satellite_drift_direction) * c.docking.satellite_velocity_mps
            r.check("[DOCK0] Satellite free-floating, no rotation",
                    float(np.linalg.norm(sat_v0 - sat_cmd)) < 1e-4
                    and float(np.linalg.norm(self.task._satellite.data.root_com_ang_vel_w[0].cpu().numpy())) < 1e-6,
                    f"satellite v {np.round(sat_v0, 5).tolist()} m/s (commanded {np.round(sat_cmd, 5).tolist()}), w = 0")
            r.check("[DOCK0] Probe fits the thruster", geo.probe.tip_radius < geo.nozzle.inner_radius(t.cfg.docking.dock_depth) - c.docking.min_wall_clearance_m,
                    f"probe tip radius {geo.probe.tip_radius:.3f} m, nozzle inner radius at the dock depth {geo.nozzle.inner_radius(t.cfg.docking.dock_depth):.3f} m")
            print(f"[INIT] PROBE_DOCK_POINT (MEP body): tip {np.round(geo.probe_dock.pos, 4).tolist()}, axis {np.round(geo.probe_dock.rot[:, 2], 4).tolist()}, tip radius {geo.probe.tip_radius:.3f} m", flush=True)
            print(f"[INIT] SAT_DOCK_POINT (satellite body): pos {np.round(geo.sat_dock.pos, 4).tolist()}, axis {np.round(geo.sat_dock.rot[:, 2], 4).tolist()}, dock_depth {t.cfg.docking.dock_depth} m", flush=True)
            if c.docking.skip_capture:
                if self.skip_capture_to_docking():
                    self.begin_docking()
                    self.goto(State.DOCK_TARGET_ACQUIRE)
                else:
                    self.goto(State.DOCK_FAILED, "skip_capture could not attach the MEP at the nominal grasp pose")

    def idle(self):
        """GUI after a successful run: keep simulating (MEP held) until the window closes."""
        sim, scene = self.sim, self.scene
        self.q_hold = self.arm.joint_pos().clone()
        print("[DEMO] Scenario finished -- the simulation keeps running with the MEP held. Close the Isaac Sim window to exit.", flush=True)
        n = 0
        with torch.no_grad():
            while self.sim_app.is_running():
                if not sim.is_playing():
                    sim.render()
                    continue
                self.hold()
                scene.write_data_to_sim()
                sim.step(render=False)
                n += 1
                if n % self.render_interval == 0:
                    sim.render()
                scene.update(dt=self.dt)

    def step(self):
        """One control step (called before every physics step)."""
        c = self.cfg
        a = c.approach
        self.state_time += self.dt
        s = self.state

        if self.ros is not None:
            self.ros.poll()
            if self.ros.abort_requested and s not in TERMINAL:
                self.goto(State.ABORTED, "ROS cmd/abort")
                s = self.state

        if s not in TERMINAL:
            err = self.check_physics()
            if err is not None:
                self.goto(State.PHYSICS_ERROR, err)
                s = self.state

        ## Tag loss in the tracking states: keep the last valid pose, stop after timeout
        if s in TRACKING | {State.PREDICTING} and self.estimate_age() > c.vision.tag_loss_timeout_s:
            self.goto(State.TAG_LOST, f"no valid constellation pose for {self.estimate_age():.2f} s (last: {self.last_vision.reason if self.last_vision else 'none'})")
            s = self.state
        if self.scenario == "dynamic" and s in TRACKING and self.sim_time > c.test.dynamic_timeout_sec:
            self.goto(State.APPROACH_TIMEOUT, f"no capture within {c.test.dynamic_timeout_sec:.0f} s")
            s = self.state

        if self.six_dof and self._w_frame_result is None and self._w_frame_ref is not None and self.sim_time - self._w_frame_ref[0] >= 1.0:
            self.check_angular_velocity_frame()

        v_est = self.predictor.velocity()
        if s == State.INIT:
            self.hold()
            if self._observe_only:
                # --orbit-only: the arm holds the observation pose, both bodies drift, nothing is captured
                if not self._orbit_view_saved and self.state_time >= 3.0:
                    self._orbit_view_saved = True
                    self.save_orbit_view_image()
                if self.state_time >= self.orbit_cfg.observe_only_duration_s:
                    self.goto(State.SUCCESS, f"orbit-only scene check over ({self.orbit_cfg.observe_only_duration_s:.0f} s)")
            elif self.state_time >= 0.5:
                self.goto(State.SEARCH)
        elif s == State.SEARCH:
            if a.start_at_observe_pose and a.start_yaw_offset_deg == 0.0:
                self.hold()
            else:
                # swing towards the configured observation pose until tags are seen
                self.q_hold = None
                self.track(self.observe_pose, a.search_speed_mps, None,
                           max_step=a.search_max_joint_step_rad, lag_limit=a.search_lag_m)
            # A swung start: the first tags appear at the edge of the image while the arm is
            # still mid-swing. Ending the search there stops the arm with only 1-2 of the 4
            # tags in view, so no pose can be estimated (measured: tags [0, 1] seen 600 mm off
            # the axis). The search therefore only ends once the arm has reached the
            # observation pose, where the whole constellation is in view.
            arrived = True
            if a.start_at_observe_pose and a.start_yaw_offset_deg != 0.0:
                pe, ae = pose_errors(self.ee_pose(), self.observe_pose)
                arrived = pe < 0.03 and ae < 2.0
            if self.last_vision is not None and self.last_vision.detections and arrived:
                self.goto(State.TAG_DETECTED, f"tags {sorted(self.last_vision.detections)}")
            elif self.state_time > a.stage_timeout_s:
                self.goto(State.TAG_LOST, "no AprilTag detected while searching")
        elif s == State.TAG_DETECTED:
            self.hold()
            if self.last_valid is not None:
                self.goto(State.POSE_ESTIMATED, f"{self.last_valid.inliers} PnP inliers, RMS {self.last_valid.reproj_rms:.3f} px")
            elif self.state_time > 3.0:
                self.goto(State.POSE_INVALID, f"no valid constellation pose ({self.last_vision.reason if self.last_vision else 'no image'})")
        elif s == State.POSE_ESTIMATED:
            self.hold()
            pe, ae = pose_errors(self.gt_cylinder(), self.last_valid.t_w_y)
            print(f"[VISION] first Cylinder_01 estimate: pos {np.round(self.last_valid.t_w_y.pos, 4).tolist()} (GT error {pe*1000:.2f} mm / {ae:.3f} deg)", flush=True)
            self.goto(State.PREDICTING)
        elif s == State.PREDICTING:
            self.hold()
            if v_est is not None and self.ros is not None and not self.ros.start_received:
                if not self._start_wait_logged:
                    self._start_wait_logged = True
                    print("[ROS] prediction ready, waiting for cmd/start (the arm holds the observation pose)", flush=True)
            elif v_est is not None:
                print(f"[PREDICT] velocity estimate {np.round(v_est, 5).tolist()} m/s from {len(self.predictor.history)} samples", flush=True)
                self.ref = self.ee_pose()
                self.q_hold = None
                self.goto(State.STATIC_MEASURE if self.scenario == "static" else State.APPROACHING)
            elif self.state_time > self.cfg.prediction.velocity_window_sec + 3.0:
                self.goto(State.PREDICTION_INVALID, f"no plausible velocity from {len(self.predictor.history)} samples")
        elif s == State.APPROACHING:
            self.step_approaching(v_est)
        elif s == State.SLOW_APPROACH:
            self.step_slow_approach(v_est)
        elif s == State.CAPTURE_ATTEMPT:
            self.step_capture_attempt(v_est)
        elif s == State.CAPTURED:
            self.hold()
            self.goto(State.HOLDING)
        elif s == State.HOLDING:
            self.step_holding()
        elif s == State.RETREAT:
            self.step_retreat()
        elif s == State.STATIC_MEASURE:
            self.step_static(v_est)
        elif s in DOCKING_STATES:
            m = self.dock_metrics()
            if self._dock_started is None:
                self._dock_started = self.sim_time
            elif (self.sim_time - self._dock_started > c.docking.phase_timeout_s
                  and s not in (State.DOCKED, State.DOCK_HOLDING) and s not in ORBIT_STATES):
                return self.goto(State.DOCK_FAILED,
                                 f"the docking phase did not finish within {c.docking.phase_timeout_s:.0f} s "
                                 f"(state {s.value}, remaining {m['geometry_distance']:.3f} m, lateral {m['lateral']*1000:.1f} mm)")
            self._tip_history.append((self.sim_time, self.probe_world().pos.copy()))
            self._tip_history = [x for x in self._tip_history if x[0] >= self.sim_time - 3.0 * self.cfg.docking.settle_window_s]
            if s == State.DOCK_TARGET_ACQUIRE:
                self.step_dock_target_acquire(m)
            elif s == State.PRE_DOCK_APPROACH:
                self.step_pre_dock(m)
            elif s == State.XY_ALIGN:
                self.step_xy_align(m)
            elif s == State.ORIENTATION_ALIGN:
                self.step_orientation_align(m)
            elif s == State.ALIGNMENT_CHECK:
                self.step_alignment_check(m)
            elif s == State.Z_APPROACH:
                self.step_z_approach(m, final=False)
            elif s == State.FINAL_INSERTION:
                self.step_z_approach(m, final=True)
            elif s == State.DOCK_READY:
                self.step_dock_ready(m)
            elif s == State.DOCKED:
                self.step_docked(m)
            elif s == State.ORBIT_TARGET_ACQUIRE:
                self.step_orbit_target_acquire(m)
            elif s == State.ORBIT_TRANSFER:
                self.step_orbit_transfer(m)
            elif s == State.ORBIT_ARRIVAL_CHECK:
                self.step_orbit_arrival_check(m)
            elif s == State.ORBIT_HOLDING:
                self.step_orbit_holding(m)
            else:
                self.step_dock_holding(m)
            if self.sim_time >= self._next_dock_log_t:
                self._next_dock_log_t = self.sim_time + 1.0 / self.cfg.logging.rate_hz - 1e-9
                self.log_dock_row(m)
                if self.cfg.logging.debug_draw:
                    if self.state in ORBIT_STATES:
                        self.update_orbit_visuals()
                    else:
                        self.update_dock_visuals(m)
            if self.sim_time >= self._next_status_t:
                self._next_status_t = self.sim_time + 2.0
                if self.state in ORBIT_STATES:
                    self.orbit_status()
                else:
                    self.dock_status(m)
            if self.sim_time >= self._next_probe_rgb_t and self.cfg.probe_camera.rgb_every_sec > 0.0:
                self._next_probe_rgb_t = self.sim_time + self.cfg.probe_camera.rgb_every_sec
                self.save_probe_rgb()
        else:  # SUCCESS / failures: hold the joints
            self.hold()
            if self.state_time >= (0.5 if s == State.SUCCESS else 1.0):
                self.done = True

    def skip_capture_to_docking(self) -> bool:
        """`docking.skip_capture`: put the MEP at its nominal pose, close the arm on it
        kinematically and attach, so a docking run does not re-run the 30 s capture.

        Nothing is teleported once the simulation is running: this happens in `start()`,
        before the first physics step, and only when the docking phase is enabled.
        """
        mep_nominal = self.task.mep_nominal
        drift = self.cfg.mep.linear_velocity_w()
        root = torch.zeros((1, 13), device=self.mep.device)
        root[0, :3] = torch.tensor(mep_nominal.pos, device=self.mep.device)
        root[0, 3:7] = torch.tensor(mep_nominal.quat, device=self.mep.device)
        root[0, 7:10] = torch.tensor(drift, device=self.mep.device)
        self.mep.write_root_state_to_sim(root)
        self.sim.forward()
        self.mep.update(1e-6)
        goal = self.mep_frame() @ self.geo.ee_contact_to_mep.inv()
        ok, q, pe, ae = self.arm.solve_kinematic(goal, self.arm.joint_pos().clone())
        self.results.check("[DOCK0] Grasp pose reachable (skip_capture)", ok,
                           f"IK err {pe*1000:.3f} mm / {math.degrees(ae):.4f} deg at the nominal MEP grasp face")
        if not ok:
            return False
        self.task._robot.set_joint_position_target(q, joint_ids=self.arm.joint_ids)
        self.q_hold = q.clone()
        self.capture.attach(0)
        self.results.check("[DOCK0] MEP attached (skip_capture)", self.capture.is_attached(0),
                           "capture FixedJoint created at the nominal grasp pose")
        if not self.capture.is_attached(0):
            return False
        self._rel_at_capture = self.ee_pose().inv() @ self.mep_frame()
        self._mep_at_capture = self.mep_frame()
        self._normal_at_capture = self.mep_grasp_normal()
        print(f"[DOCK] skip_capture: MEP attached at the nominal grasp pose (IK err {pe*1000:.2f} mm), "
              f"capture phase skipped", flush=True)
        return True

    def mep_grasp_normal(self) -> np.ndarray:
        return (self.mep_frame() @ self.geo.mep_grasp).rot[:, 2]

    def after_capture_state(self) -> State:
        """Where the capture phase hands over: SUCCESS, or the docking phase."""
        if not self.cfg.docking.enabled:
            return State.SUCCESS
        self.begin_docking()
        return State.DOCK_TARGET_ACQUIRE

    def check_angular_velocity_frame(self):
        """six_dof, ~1 s after the start (GT, judgement only): is the configured /
        reported angular velocity a WORLD-frame vector? Compares Log(R1 R0^T) / dt
        (world) and Log(R0^T R1) / dt (body) with the commanded w."""
        t0, f0 = self._w_frame_ref
        f1 = self.mep_frame()
        dt = self.sim_time - t0
        w_cmd = self.cfg.mep.angular_velocity_w()
        w_world = so3_log(f1.rot @ f0.rot.T) / dt
        w_body = so3_log(f0.rot.T @ f1.rot) / dt
        w_meas = self.mep.data.root_com_ang_vel_w[0].cpu().numpy()
        err_w, err_b = float(np.linalg.norm(w_world - w_cmd)), float(np.linalg.norm(w_body - w_cmd))
        tol = 0.05 * float(np.linalg.norm(w_cmd)) + 1e-5
        distinct = float(np.linalg.norm(w_world - w_body)) > 2.0 * tol
        # Frame question = DIRECTION: which of the two readings of the commanded vector
        # (world / body) is the one the body actually rotates about. The rate is judged
        # separately (`rate_ratio`): PhysX may report a different |w| than the pose change.
        n_cmd = float(np.linalg.norm(w_cmd))
        n_world = float(np.linalg.norm(w_world))
        cos = float(w_world @ w_cmd) / max(1e-12, n_world * n_cmd)
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
        ratio = n_world / max(1e-12, n_cmd)
        self._w_frame_result = {
            "dt_s": dt, "w_cmd": w_cmd.tolist(), "w_from_pose_world": w_world.tolist(), "w_from_pose_body": w_body.tolist(),
            "w_root_com_ang_vel_w": w_meas.tolist(), "err_world": err_w, "err_body": err_b, "tolerance": tol,
            "world_and_body_distinguishable": distinct,
            "direction_error_deg": angle_deg, "rate_ratio_pose_over_cmd": ratio,
            "rate_ratio_pose_over_root_com": n_world / max(1e-12, float(np.linalg.norm(w_meas))),
            "pass": angle_deg < 5.0 and (err_w < err_b or not distinct),
        }
        note = "" if abs(ratio - 1.0) <= 0.05 else f"; WARNING: rotation rate from poses is {ratio:.3f} x commanded (root_com_ang_vel_w {float(np.linalg.norm(w_meas)) / n_cmd:.3f} x)"
        print(f"[FRAME] angular velocity over {dt:.2f} s: from poses (world) {np.round(w_world, 5).tolist()}, (body) {np.round(w_body, 5).tolist()}, "
              f"root_com_ang_vel_w {np.round(w_meas, 5).tolist()}, commanded {np.round(w_cmd, 5).tolist()} rad/s -> "
              f"{'WORLD frame confirmed' if self._w_frame_result['pass'] else 'MISMATCH'} (direction error {angle_deg:.2f} deg){note}"
              f"{'' if distinct else ' (world/body not distinguishable for this start orientation)'}", flush=True)

    ## Stage A/B: far approach, then track the predicted target at the approach standoff
    def step_approaching(self, v_est):
        a = self.cfg.approach
        pred = self.predicted()
        if pred is None:
            return self.goto(State.PREDICTION_INVALID, "prediction unavailable while approaching")
        self.standoff = a.approach_standoff_m
        self.goal = self.ee_goal(pred @ self.t_y_t, self.standoff)
        v_ff, w_ff = self.feedforward()
        self.track(self.goal, a.speed_far_mps, v_ff, w_ff=w_ff)
        if self._settled(a.settle_lateral_m, a.settle_angle_deg, 0.02, 0.5):
            m = self.est_capture_metrics()
            print(f"[APPROACH] Stage B: tracking the predicted Cylinder_01 at {self.standoff:.2f} m (lateral {m['lateral']*1000:.1f} mm, angle {m['angle']:.2f} deg)", flush=True)
            self.goto(State.SLOW_APPROACH)
        elif self.state_time > a.stage_timeout_s:
            self.goto(State.APPROACH_TIMEOUT, "approach standoff not reached")

    ## Stage C/D: close the standoff slowly while staying on the predicted axis
    def step_slow_approach(self, v_est):
        a = self.cfg.approach
        c = self.cfg.capture
        pred = self.predicted()
        if pred is None:
            return self.goto(State.PREDICTION_INVALID, "prediction unavailable in slow approach")
        # Gate on the EE pose w.r.t. the current Cylinder_01 estimate (as the capture
        # conditions are): on the axis, aligned, and not lagging the commanded standoff
        m = self.est_capture_metrics()
        gate = m["lateral"] < a.gate_lateral_m and m["angle"] < a.gate_angle_deg and abs(m["gap"] - self.standoff) < 0.02
        # Relative velocity guard: do not close in while the EE moves too fast relative to the MEP
        if not math.isnan(m["rel_vel"]) and m["rel_vel"] > 0.8 * c.max_relative_velocity_mps:
            gate = False
        if gate and self.standoff > a.final_gap_m:
            span = max(1e-6, a.near_distance_m - a.capture_range_m)
            f = min(1.0, max(0.0, (self.standoff - a.capture_range_m) / span))
            v = a.speed_capture_range_mps + (a.speed_near_mps - a.speed_capture_range_mps) * f
            if self.standoff > a.near_distance_m:
                v = a.speed_near_mps
            self.standoff = max(a.final_gap_m, self.standoff - v * self.dt)
        if self.standoff <= a.capture_range_m and not self._capture_range_logged:
            self._capture_range_logged = True
            print(f"[APPROACH] Stage D: capture range entered (standoff {self.standoff:.3f} m, distance to Cylinder_01 {m['distance']:.3f} m)", flush=True)
        self.goal = self.ee_goal(pred @ self.t_y_t, self.standoff)
        v_ff, w_ff = self.feedforward()
        self.track(self.goal, a.speed_near_mps, v_ff, w_ff=w_ff)
        # Stage D -> E once the EE holds the final gap on the Cylinder_01 axis. Judged
        # against the current estimate (as the capture conditions are), not against the
        # 0.3 s-ahead goal: the stiff, overdamped joint drives trail their targets by
        # ~0.6 s, so the EE stays a few mm behind the leading goal while on the axis.
        on_axis = (m["lateral"] < a.gate_lateral_m and m["angle"] < a.gate_angle_deg
                   and abs(m["gap"] - a.final_gap_m) < 0.005)
        if self.standoff <= a.final_gap_m + 1e-9 and on_axis:
            self._settled_since = self._settled_since if self._settled_since is not None else self.sim_time
        else:
            self._settled_since = None
        if self._settled_since is not None and self.sim_time - self._settled_since >= 0.3:
            self.goto(State.CAPTURE_ATTEMPT)
        elif self.state_time > a.stage_timeout_s:
            self.goto(State.APPROACH_TIMEOUT, f"slow approach stalled at standoff {self.standoff:.3f} m")

    ## Stage E/F: check every condition, then the existing magnet attach (FixedJoint)
    def step_capture_attempt(self, v_est):
        a = self.cfg.approach
        pred = self.predicted()
        if pred is not None:
            self.goal = self.ee_goal(pred @ self.t_y_t, a.final_gap_m)
            v_ff, w_ff = self.feedforward()
            self.track(self.goal, a.speed_capture_range_mps, v_ff, w_ff=w_ff)
        if self.ros is not None and not self.ros.capture_enabled:
            if not self._capture_wait_logged:
                self._capture_wait_logged = True
                print("[ROS] capture disabled (cmd/capture_enable false): tracking without attaching", flush=True)
            return
        ok, m, failed = self.capture_conditions()
        if not ok:
            if self.state_time > 5.0:
                self.goto(State.CAPTURE_FAILED, "; ".join(failed))
            return
        v_gt, _ = self.gt_mep_velocity_at(self.ee_pose().pos)
        m_gt = self.gt_capture_metrics()
        print(f"[CAPTURE] ATTEMPT  t={self.sim_time:.2f} s | vision: distance {m['distance']:.4f} m, gap {m['gap']*1000:.1f} mm, lateral {m['lateral']*1000:.2f} mm, "
              f"angle {m['angle']:.3f} deg, rel. velocity {m['rel_vel']:.4f} m/s, rel. angular velocity {m['rel_ang_vel']:.4f} rad/s | GT: distance {m_gt['distance']:.4f} m, "
              f"lateral {m_gt['lateral']*1000:.2f} mm, angle {m_gt['angle']:.3f} deg, rel. velocity {m_gt['rel_vel']:.4f} m/s, "
              f"rel. angular velocity {m_gt['rel_ang_vel']:.4f} rad/s", flush=True)
        self.capture.attach(0)  # existing magnet-style capture -> UsdPhysics.FixedJoint
        if self.capture.is_attached(0):
            print(f"[CAPTURE] CAPTURED  t={self.sim_time:.2f} s  (FixedJoint {self.capture._joint_prim_paths[0]})", flush=True)
            self._capture_info = {
                "time_s": self.sim_time,
                "vision": m,
                "gt": m_gt,
                "ee_speed_mps": float(np.linalg.norm(self.ee_velocity())),
                "mep_speed_mps": float(np.linalg.norm(v_gt)),
            }
            self._rel_at_capture = self.ee_pose().inv() @ self.mep_frame()
            self._mep_at_capture = self.mep_frame()
            self._normal_at_capture = (self.estimate_now() @ self.t_y_t).rot[:, 2]
            self.q_hold = self.arm.joint_pos().clone()
            self.goto(State.CAPTURED)
        else:
            self.goto(State.CAPTURE_FAILED, "FixedJoint was not created")

    def relative_drift(self) -> Tuple[float, float]:
        now = self.ee_pose().inv() @ self.mep_frame()
        pe, ae = pose_errors(self._rel_at_capture, now)
        return pe, ae

    def step_holding(self):
        self.hold()
        h = self._hold
        pe, ae = self.relative_drift()
        h["max_drift_mm"] = max(h.get("max_drift_mm", 0.0), pe * 1000.0)
        h["max_drift_deg"] = max(h.get("max_drift_deg", 0.0), ae)
        v = float(torch.norm(self.mep.data.root_com_lin_vel_w[0]))
        w = math.degrees(float(torch.norm(self.mep.data.root_com_ang_vel_w[0])))
        h["max_mep_speed"] = max(h.get("max_mep_speed", 0.0), v)
        if self.state_time >= 0.5 * self.cfg.test.holding_duration_sec:
            h["max_mep_speed_second_half"] = max(h.get("max_mep_speed_second_half", 0.0), v)
        h["max_mep_rate_deg_s"] = max(h.get("max_mep_rate_deg_s", 0.0), w)
        ee = self.ee_pose()
        h.setdefault("ee_start", ee.pos.copy())
        h["max_ee_motion_mm"] = max(h.get("max_ee_motion_mm", 0.0), 1000.0 * float(np.linalg.norm(ee.pos - h["ee_start"])))
        if self.state_time >= self.cfg.test.holding_duration_sec:
            h["duration_s"] = self.state_time
            h["final_mep_speed"] = v
            h["joint_valid"] = self.capture.is_attached(0)
            h["mep_displacement_mm"] = 1000.0 * float(np.linalg.norm(self.mep_frame().pos - self._mep_at_capture.pos))
            print(f"[HOLD] {self.state_time:.1f} s held: max MEP-EE drift {h['max_drift_mm']:.3f} mm / {h['max_drift_deg']:.4f} deg, "
                  f"MEP speed now {v*1000:.2f} mm/s, FixedJoint valid {h['joint_valid']}", flush=True)
            if self.cfg.test.retreat_distance_m > 0.0:
                self.goto(State.RETREAT)
            else:
                self.goto(self.after_capture_state())

    def step_retreat(self):
        """Slowly back the EE off along the face normal: the MEP must come along."""
        c = self.cfg.test
        if self._retreat is None:
            start = self.ee_pose()
            self._retreat = {"start": start, "mep_start": self.mep_frame().pos.copy(), "dist": 0.0, "max_drift_mm": 0.0, "max_drift_deg": 0.0}
            self.ref = start
            self.q_hold = None
        rt = self._retreat
        rt["dist"] = min(c.retreat_distance_m, rt["dist"] + c.retreat_speed_mps * self.dt)
        goal = Frame(rt["start"].pos + rt["dist"] * self._normal_at_capture, rt["start"].rot)
        self.goal = goal
        self.ref = goal
        q = self.arm.ik_joint_target(goal, 0.01)
        self.task._robot.set_joint_position_target(q, joint_ids=self.arm.joint_ids)
        self.set_velocity_feedforward(None)
        pe, ae = self.relative_drift()
        rt["max_drift_mm"] = max(rt["max_drift_mm"], pe * 1000.0)
        rt["max_drift_deg"] = max(rt["max_drift_deg"], ae)
        if rt["dist"] >= c.retreat_distance_m and self.state_time > c.retreat_distance_m / c.retreat_speed_mps + 3.0:
            ee_moved = float((self.ee_pose().pos - rt["start"].pos) @ self._normal_at_capture)
            mep_moved = float((self.mep_frame().pos - rt["mep_start"]) @ self._normal_at_capture)
            rt["ee_moved_m"], rt["mep_moved_m"] = ee_moved, mep_moved
            print(f"[RETREAT] EE moved {ee_moved*1000:.1f} mm, MEP moved {mep_moved*1000:.1f} mm along the face normal, "
                  f"max MEP-EE drift {rt['max_drift_mm']:.3f} mm / {rt['max_drift_deg']:.4f} deg", flush=True)
            self.q_hold = None
            self.goto(self.after_capture_state())

    def step_static(self, v_est):
        """Test 1: stationary MEP observed from each configured standoff."""
        c = self.cfg
        standoffs = c.test.static_standoffs_m
        if self._static_idx >= len(standoffs):
            self.hold()
            self.goto(State.SUCCESS)
            return
        y = self.estimate_now()
        s = standoffs[self._static_idx]
        if self.goal is None or self._settled_since is None:
            self.goal = self.ee_goal(y @ self.t_y_t, s)
        self.track(self.goal, c.approach.speed_far_mps, None)
        # Sample only once the whole estimation window was observed from this standoff
        if not self._settled(0.005, 0.3, 0.005, max(c.prediction.velocity_window_sec, c.prediction.orientation_window_sec) + 0.2):
            if self.state_time > c.approach.stage_timeout_s:
                self.goto(State.APPROACH_TIMEOUT, f"static pose at {s} m not reached")
            return
        lv = self.last_vision
        if lv is not None and lv.valid and lv.t > self._static_last_t:
            self._static_last_t = lv.t
            gt = self.gt_cylinder()
            pe, ae = pose_errors(gt, self.estimate_now())  # system estimate (window-fused)
            rpe, rae = pose_errors(gt, lv.t_w_y)  # this image alone
            self._static_samples.append((pe * 1000.0, ae, lv.reproj_rms, rpe * 1000.0, rae))
        if len(self._static_samples) >= c.test.static_samples:
            arr = np.array(self._static_samples)
            dist = float(np.linalg.norm(self.cam_pose().pos - self.gt_cylinder().pos))
            stat = {
                "standoff_m": s,
                "camera_to_cylinder01_m": dist,
                "samples": len(arr),
                "position_error_mm_mean": float(arr[:, 0].mean()),
                "position_error_mm_max": float(arr[:, 0].max()),
                "angle_error_deg_mean": float(arr[:, 1].mean()),
                "angle_error_deg_max": float(arr[:, 1].max()),
                "reproj_rms_px_mean": float(arr[:, 2].mean()),
                "single_image_position_error_mm_mean": float(arr[:, 3].mean()),
                "single_image_position_error_mm_max": float(arr[:, 3].max()),
                "single_image_angle_error_deg_mean": float(arr[:, 4].mean()),
                "single_image_angle_error_deg_max": float(arr[:, 4].max()),
            }
            self._static_stats.append(stat)
            print(f"[STATIC] standoff {s:.2f} m (camera-Cylinder_01 {dist:.3f} m): position error mean {stat['position_error_mm_mean']:.3f} / max "
                  f"{stat['position_error_mm_max']:.3f} mm, angle error mean {stat['angle_error_deg_mean']:.4f} / max {stat['angle_error_deg_max']:.4f} deg "
                  f"| single image: {stat['single_image_position_error_mm_max']:.3f} mm / {stat['single_image_angle_error_deg_max']:.4f} deg max", flush=True)
            self._static_samples = []
            self._static_idx += 1
            self.goal = None
            self._settled_since = None

    _static_last_t = -1.0

    ###############################################
    ### Phase 2: Ares1 probe -> satellite dock ###
    ###############################################
    ## The controlled frame is the probe tip (PROBE_DOCK_POINT, measured from the USD by
    ## `docking.py`). The relative pose comes from the *simulated body poses* of the MEP
    ## and the satellite, i.e. it is transform/ground-truth based, NOT a vision estimate:
    ## no camera measures the satellite here. `cam_probe` depth is an independent
    ## cross-check of the docking-axis distance (`probe_dock.depth_valid`), never the
    ## only source, and the probe never advances on an invalid depth reading.

    def probe_world(self) -> Frame:
        """Live PROBE_DOCK_POINT: tip + insertion axis, from the MEP body pose."""
        return self.mep_frame() @ self.geo.probe_dock

    def dock_world(self) -> Frame:
        """Live SAT_DOCK_POINT: `dock_depth` inside the nozzle, +Z into the nozzle."""
        return self.sat_frame() @ self.geo.sat_dock

    def sat_frame(self) -> Frame:
        d = self.task._satellite.data
        return Frame.from_pos_quat(d.root_pos_w[0].tolist(), d.root_quat_w[0].tolist())

    def sat_exit_world(self) -> Frame:
        return self.sat_frame() @ self.geo.sat_exit

    def sat_velocity_at(self, p) -> np.ndarray:
        d = self.task._satellite.data
        v = d.root_com_lin_vel_w[0].cpu().numpy()
        w = d.root_com_ang_vel_w[0].cpu().numpy()
        c = d.root_com_pos_w[0].cpu().numpy()
        return v + np.cross(w, np.asarray(p, dtype=float) - c)

    def tool_velocity_vec(self) -> np.ndarray:
        """Velocity of the IK tool point [m/s] (world), from proprioception."""
        lid = self.arm.link_id
        v = self.task._robot.data.body_lin_vel_w[0, lid].cpu().numpy()
        w = self.task._robot.data.body_ang_vel_w[0, lid].cpu().numpy()
        return v + np.cross(w, self.arm.link_pose().rot @ self.arm.tool.pos)

    def probe_velocity(self) -> np.ndarray:
        """Velocity of the probe tip [m/s] (the MEP is rigid on the arm once captured)."""
        return self.gt_mep_velocity_at(self.probe_world().pos)[0]

    def read_probe_depth(self) -> Tuple[float, int]:
        """(median depth [m], valid pixels) at the principal point of `cam_probe`."""
        d = self.cfg.docking
        if self._probe_cam is None or not d.depth_enabled:
            return math.nan, 0
        out = self._probe_cam.data.output
        if "distance_to_image_plane" not in out:
            return math.nan, 0
        img = out["distance_to_image_plane"][0].cpu().numpy()
        return probe_dock.sample_depth(np.squeeze(img), self._probe_k, d.depth_patch_px, d.depth_min_m, d.depth_max_m)

    def dock_metrics(self) -> Dict[str, float]:
        """Relative pose, depth cross-check and clearance at the current instant."""
        d = self.cfg.docking
        probe, dock = self.probe_world(), self.dock_world()
        m = probe_dock.dock_errors(probe, dock)
        m["roll_rpy_deg"], m["pitch_rpy_deg"], m["yaw_rpy_deg"] = probe_dock.rpy_errors_deg(probe, dock)
        m["insertion_depth"] = probe_dock.insertion_depth(probe, self.sat_exit_world())
        m["geometry_distance"] = -m["axial"]  # remaining insertion distance [m]
        raw, px = self.read_probe_depth()
        m["depth_raw"], m["depth_pixels"] = raw, float(px)
        offset = self._depth_offset if self._depth_offset is not None else d.depth_surface_offset_m
        m["depth_distance"] = probe_dock.depth_to_dock_distance(raw, self._cam_to_tip, offset)
        ok, why = probe_dock.depth_valid(d, m["depth_distance"], m["geometry_distance"])
        m["depth_ok"], self._depth_reason = float(ok), why
        rel_v = self.probe_velocity() - self.sat_velocity_at(probe.pos)
        m["rel_vx"], m["rel_vy"], m["rel_vz"] = (float(x) for x in rel_v)
        m["rel_speed"] = float(np.linalg.norm(rel_v))
        # Probe-to-wall gap at the current depth (only meaningful inside the nozzle)
        inner = self.geo.nozzle.inner_radius(max(0.0, m["insertion_depth"]))
        m["clearance"] = probe_dock.wall_clearance(inner, self.geo.probe.tip_radius, m["lateral"], m["insertion_depth"])
        return m

    def docking_tracking_valid(self) -> Tuple[bool, str]:
        """Fail-safe: the probe/dock frames must be finite and the MEP still held."""
        if not self.capture.is_attached(0):
            return False, "the MEP is no longer attached to the arm"
        probe, dock = self.probe_world(), self.dock_world()
        if not (np.isfinite(probe.pos).all() and np.isfinite(dock.pos).all()):
            return False, "probe tip or docking target pose is not finite"
        prev, prev_t = self._dock.get("prev_dock_pos"), self._dock.get("prev_dock_t", self.sim_time)
        if prev is not None:
            # The check is not made every step (the transport takes a minute), so the drift of a
            # moving satellite over the elapsed time is expected motion, not a jump
            expected = self.sat_velocity_at(dock.pos) * (self.sim_time - prev_t)
            jump = float(np.linalg.norm(dock.pos - prev - expected))
            if jump > 0.25:  # the target cannot move that far in one control step
                return False, f"docking target jumped {jump*1000:.0f} mm in one step"
        self._dock["prev_dock_pos"] = dock.pos.copy()
        self._dock["prev_dock_t"] = self.sim_time
        return True, "ok"

    def begin_docking(self):
        """Hand over from the capture phase: the IK tool becomes the probe tip."""
        d = self.cfg.docking
        link = self.arm.link_pose()
        tip_in_link = link.inv() @ self.probe_world()
        self.arm.set_tool(tip_in_link)
        self._dock_ref = self.arm.tool_pose()
        self.q_hold = None
        m = self.dock_metrics()
        print(f"[DOCK] probe tip is now the controlled frame (in link '{self.cfg.camera.name}' parent): "
              f"pos {np.round(tip_in_link.pos, 4).tolist()} m", flush=True)
        print(f"[DOCK] target acquired: remaining insertion {m['geometry_distance']:.3f} m, lateral {m['lateral']*1000:.1f} mm, "
              f"axis {m['axis_deg']:.2f} deg, roll {m['roll_deg']:.2f} deg, depth raw {m['depth_raw']:.3f} m ({int(m['depth_pixels'])} px)", flush=True)
        self.frame_docking_view(close=False)
        self.results.metrics["docking"] = {
            "probe_tip_in_link": {"pos": tip_in_link.pos.tolist(), "quat_wxyz": list(tip_in_link.quat)},
            "at_acquire": {k: v for k, v in m.items() if isinstance(v, float)},
            "pre_dock_axial_m": self._pre_dock_axial(),
        }

    def _pre_dock_axial(self) -> float:
        """Axial coordinate of the pre-dock pose: `pre_dock_distance_m` in front of the
        nozzle *exit*, i.e. `dock_depth + pre_dock_distance` before SAT_DOCK_POINT."""
        return -(float(self.task.cfg.docking.dock_depth) + float(self.cfg.docking.pre_dock_distance_m))

    def track_probe(self, goal: Frame, speed: float, max_step: Optional[float] = None, follow_satellite: bool = True,
                    hold_tol_m: Optional[float] = None, lag_limit_m: Optional[float] = None, accel_mps2: Optional[float] = None):
        """Rate-limited probe-tip tracking with the satellite's motion fed forward.

        Same structure as the capture-phase `track()`, but the controlled frame is the
        probe tip and the feed-forward is the docking target's velocity, so a drifting
        satellite does not leave a standing tracking error.
        """
        d = self.cfg.docking
        if self._dock_ref is None:
            self._dock_ref = self.arm.tool_pose()
        prev = self._dock_ref
        # Once docked the satellite rides on the arm: its velocity is the arm's own, not a target motion
        pos = prev.pos + (self.sat_velocity_at(self.dock_world().pos) * self.dt if follow_satellite else 0.0)
        rot = prev.rot
        # The reference advances continuously towards the goal and then stops there.
        # It deliberately does NOT gate on the measured tracking error: an on/off gate
        # advances the reference on the half-swing that closes the error and freezes it
        # on the other, which rectifies the arm + 3 t payload mode (period ~20 s) and
        # pumps it -- measured as a steady +-150 mm lateral swing that never decayed.
        ramp_cfg = d if accel_mps2 is None else dataclasses.replace(d, accel_mps2=accel_mps2)
        self._dock_speed = probe_dock.ramped(ramp_cfg, self._dock_speed, speed, self.dt)
        step_speed = self._dock_speed
        if lag_limit_m is not None:
            # A heavy payload (MEP + Client) follows far slower than the reference: once the reference is ahead by
            # more than a joint step can pull, the clamped joint step no longer points at the goal (measured: the
            # Client stuck ~0.25 m short of the orbit). The reference speed therefore fades out linearly with the
            # measured lag (continuous, not an on/off gate, so it does not rectify the swing mode).
            # Only the lag along the direction of travel counts: the joined bodies also keep drifting sideways
            # (along the orbit) and that must not stop the reference.
            to_goal = goal.pos - prev.pos
            dist = float(np.linalg.norm(to_goal))
            lag = max(0.0, float((prev.pos - self.arm.tool_pose().pos) @ (to_goal / dist))) if dist > 1e-9 else 0.0
            step_speed *= max(0.0, 1.0 - lag / lag_limit_m)
            self._dock["tool_lag_m"] = lag
        delta = goal.pos - pos
        n = float(np.linalg.norm(delta))
        if n > 1e-9:
            pos = pos + delta / n * min(n, step_speed * self.dt)
        ang = rotation_angle(rot, goal.rot)
        if ang > 1e-9:
            f = min(1.0, math.radians(d.align_speed_deg_s) * self.dt / ang)
            rot = interp_frame(Frame(pos, rot), Frame(pos, goal.rot), f).rot
        self._dock_ref = Frame(pos, rot)
        n = float(np.linalg.norm(goal.pos - pos))  # distance still to run after this step
        # Once the reference has arrived, latch the joint target instead of re-deriving
        # it from the measured joints every step: `ik_joint_target` returns
        # `q_measured + lead`, so with the tip at its goal (lead -> 0) the target would
        # follow the joints and the arm would hold no pose at all, leaving the payload
        # swing undamped. Latching is the same mechanism the capture phase uses (`hold`).
        # (Integrating the target instead was tried and destabilised the arm: with a
        # joint stiffness of 40000 N m/rad even a 0.2 rad accumulated offset is huge.)
        # A reference that still advances with a drifting satellite is moving even when it has caught up
        # with the goal: `hold()` (latched joints, zero velocity target) would stop the arm while the
        # target keeps drifting away (measured: a steady ~40 mm / 4 s lag behind a 10 mm/s satellite)
        moved = n > 1e-9 or ang > 1e-9 or float(np.linalg.norm(pos - prev.pos)) > 1e-9
        if not moved and hold_tol_m is not None:
            # The reference has stopped, but a heavy payload (MEP + Client) can still lag far behind it. Latching
            # the joints now would freeze that lagging pose (measured: the Client stuck ~0.3 m short of the orbit,
            # swinging about it), so the arm keeps being pulled to the reference until it is really there.
            moved = float(np.linalg.norm(self.arm.tool_pose().pos - pos)) > hold_tol_m
        if not moved:
            return self.hold()
        self.q_hold = None
        step = self.cfg.approach.max_joint_step_rad if max_step is None else max_step
        q = self.arm.ik_joint_target(self._dock_ref, step)
        self.task._robot.set_joint_position_target(q, joint_ids=self.arm.joint_ids)
        # Feed the reference's own velocity forward. The joint drives are PD on position
        # *and* velocity, so a zero velocity target brakes against the moving position
        # target (measured: the probe tip crept at ~20 mm/s with a 100 mm/s reference).
        v_cmd = (pos - prev.pos) / self.dt
        # Active damping: bias the joint velocity target against the measured tip
        # velocity error so the drives brake the lightly damped arm + payload swing
        # instead of only holding a position target.
        v_ff = v_cmd - d.swing_damping * (self.tool_velocity_vec() - v_cmd)
        self.set_velocity_feedforward(v_ff, so3_log(rot @ prev.rot.T) / self.dt)

    def dock_hold(self):
        """Stop the probe where it is (fail-safe / waiting states)."""
        self._dock_ref = self.arm.tool_pose()
        self.hold()

    def _dock_stage_timeout(self, m) -> bool:
        if self.state_time <= self.cfg.docking.stage_timeout_s:
            return False
        self.goto(State.DOCK_FAILED, f"{self.state.value} timed out after {self.state_time:.0f} s "
                                     f"(remaining {m['geometry_distance']:.3f} m, lateral {m['lateral']*1000:.1f} mm, axis {m['axis_deg']:.2f} deg)")
        return True

    ## States
    def step_dock_target_acquire(self, m):
        """Confirm the docking target and the probe pose, then plan the pre-dock pose."""
        self.dock_hold()
        if self.state_time < self.cfg.docking.settle_time_s:
            return
        ok, why = self.docking_tracking_valid()
        if not ok:
            return self.goto(State.DOCK_FAILED, f"docking target invalid: {why}")
        self.results.check("[DOCK1] Docking target and probe tip acquired", True,
                           f"probe tip {np.round(self.probe_world().pos, 3).tolist()}, SAT_DOCK_POINT {np.round(self.dock_world().pos, 3).tolist()}, "
                           f"remaining insertion {m['geometry_distance']:.3f} m")
        self.goto(State.PRE_DOCK_APPROACH)

    def step_pre_dock(self, m):
        """Move the probe tip onto the docking axis, `pre_dock_distance_m` in front of
        the nozzle exit (never a world-Z offset: the axis comes from the dock frame)."""
        d = self.cfg.docking
        goal = probe_dock.tip_goal(self.dock_world(), self._pre_dock_axial())
        pe, ae = pose_errors(self.arm.tool_pose(), goal)
        self._dock["commanded_speed"] = probe_dock.decelerated(d, d.transport_speed_mps, pe)
        self.track_probe(goal, self._dock["commanded_speed"])
        # Hand over only once the swing left by the transport has died out, otherwise the
        # alignment chases a tip that is still moving 100+ mm per half period
        if pe < 0.05 and self.tip_settled():
            self._dock_axial_hold = m["axial"]
            self.frame_docking_view(close=True)
            print(f"[DOCK] pre-dock reached and settled: tip {-m['insertion_depth']:.3f} m in front of the nozzle exit, "
                  f"lateral {m['lateral']*1000:.1f} mm, axis {m['axis_deg']:.2f} deg, goal error {pe*1000:.0f} mm", flush=True)
            self.goto(State.XY_ALIGN)
        else:
            self._dock_stage_timeout(m)

    def align_goal(self, m) -> Frame:
        """Tip goal for the alignment states: on the docking axis at a *frozen* axial
        distance. Using the live axial error would make the goal follow the arm's own
        oscillation, which keeps the swing alive instead of damping it."""
        if self._dock_axial_hold is None:
            self._dock_axial_hold = m["axial"]
        return probe_dock.tip_goal(self.dock_world(), self._dock_axial_hold)

    def align_track(self, m):
        """Decelerating lateral/angular correction with a reduced joint step."""
        d = self.cfg.docking
        goal = self.align_goal(m)
        pe, _ = pose_errors(self.arm.tool_pose(), goal)
        self._dock["commanded_speed"] = probe_dock.decelerated(d, d.align_speed_mps, pe)
        self.track_probe(goal, self._dock["commanded_speed"],
                         max_step=0.5 * self.cfg.approach.max_joint_step_rad)

    def tip_settled(self) -> bool:
        d = self.cfg.docking
        return probe_dock.settled(self._tip_history, self.sim_time, d.settle_window_s, d.settle_window_m)

    def step_xy_align(self, m):
        """Lateral (dock-frame X/Y) alignment at a constant docking-axis distance."""
        d = self.cfg.docking
        self.align_track(m)
        if m["lateral"] <= d.align_lateral_m:
            print(f"[DOCK] X/Y aligned: e_x {m['e_x']*1000:.2f} mm, e_y {m['e_y']*1000:.2f} mm "
                  f"(lateral {m['lateral']*1000:.2f} mm <= {d.align_lateral_m*1000:.0f} mm)", flush=True)
            self.goto(State.ORIENTATION_ALIGN)
        else:
            self._dock_stage_timeout(m)

    def step_orientation_align(self, m):
        """Roll/pitch/yaw alignment, still at a constant docking-axis distance."""
        d = self.cfg.docking
        self.align_track(m)
        if m["axis_deg"] <= d.align_axis_deg and math.isfinite(m["roll_deg"]) and m["roll_deg"] <= d.align_roll_deg:
            print(f"[DOCK] orientation aligned: axis {m['axis_deg']:.3f} deg, roll {m['roll_deg']:.3f} deg "
                  f"(rpy {m['roll_rpy_deg']:.2f} / {m['pitch_rpy_deg']:.2f} / {m['yaw_rpy_deg']:.2f} deg)", flush=True)
            self.goto(State.ALIGNMENT_CHECK)
        else:
            self._dock_stage_timeout(m)

    def step_alignment_check(self, m):
        """Both gates must hold together for `align_hold_s`, with the tip at rest,
        before any insertion."""
        d = self.cfg.docking
        self.align_track(m)
        ok, bad = probe_dock.alignment_ok(d, m)
        if ok and self._depth_offset is None and d.auto_calibrate and d.depth_enabled and m["lateral"] <= d.depth_calibration_lateral_m:
            self._calibrate_depth(m)  # only on the axis: elsewhere the ray misses the nozzle or hits a nearer structure
        if ok and not self.tip_settled():
            ok, bad = False, [f"probe tip not at rest (> {d.settle_window_m*1000:.0f} mm over {d.settle_window_s:.0f} s)"]
            self._dock_align_since = None
            return self._dock_stage_timeout(m)
        if not ok:
            self._dock_align_since = None
            if m["lateral"] > d.align_lateral_m:
                return self.goto(State.XY_ALIGN, "; ".join(bad))
            return self.goto(State.ORIENTATION_ALIGN, "; ".join(bad))
        self._dock_align_since = self._dock_align_since if self._dock_align_since is not None else self.sim_time
        if self.sim_time - self._dock_align_since >= d.align_hold_s:
            self.results.check("[DOCK2] Alignment before the docking-axis approach", True,
                               f"lateral {m['lateral']*1000:.2f} mm (<= {d.align_lateral_m*1000:.0f}), axis {m['axis_deg']:.3f} deg (<= {d.align_axis_deg}), "
                               f"roll {m['roll_deg']:.3f} deg (<= {d.align_roll_deg}), held {d.align_hold_s:.1f} s")
            print(f"[DOCK] alignment check passed; starting the docking-axis approach "
                  f"(remaining {m['geometry_distance']:.3f} m, depth {m['depth_distance']:.3f} m)", flush=True)
            self.goto(State.Z_APPROACH)
        else:
            self._dock_stage_timeout(m)

    def step_z_approach(self, m, final: bool):
        """Closed-loop approach along the docking axis.

        Every step re-reads both body poses, so a drifting satellite is tracked; the
        forward motion stops and the demo returns to aligning as soon as the corridor
        is left, and it never advances without a valid depth reading.
        """
        d = self.cfg.docking
        ok, bad = probe_dock.approach_still_aligned(d, m)
        if not ok:
            self._dock_align_since = None
            self._dock["realigns"] = int(self._dock.get("realigns", 0)) + 1
            print(f"[DOCK] approach stopped, re-aligning: {'; '.join(bad)}", flush=True)
            return self.goto(State.XY_ALIGN)
        valid, why = self.docking_tracking_valid()
        if not valid:
            return self.goto(State.DOCK_FAILED, f"docking target invalid: {why}")
        self._dock_axial_hold = None
        remaining = m["geometry_distance"]
        # The alignment gate can let the probe through while it is still swinging off the axis, where the
        # calibration is refused (`depth_calibration_lateral_m`); without it the depth stays invalid and the
        # approach would hold forever. So it is also taken here, as soon as the probe is on the axis, outside the nozzle.
        if (self._depth_offset is None and d.auto_calibrate and d.depth_enabled and m["lateral"] <= d.depth_calibration_lateral_m
                and remaining >= d.slow_zone_m):
            self._calibrate_depth(m)
            if self._depth_offset is not None:
                m["depth_distance"] = probe_dock.depth_to_dock_distance(m["depth_raw"], self._cam_to_tip, self._depth_offset)
                ok_depth, self._depth_reason = probe_dock.depth_valid(d, m["depth_distance"], m["geometry_distance"])
                m["depth_ok"] = float(ok_depth)
        # Fail-safe: hold position (do not advance) while the depth is not trusted
        depth_blocked = d.require_depth and not m["depth_ok"]
        if depth_blocked:
            self._dock["depth_blocked_s"] = float(self._dock.get("depth_blocked_s", 0.0)) + self.dt
            if not self._dock.get("depth_warned"):
                self._dock["depth_warned"] = True
                print(f"[DOCK] holding: {self._depth_reason}", flush=True)
            speed = 0.0
            axial_cmd = m["axial"]
        else:
            self._dock["depth_warned"] = False
            speed = probe_dock.approach_speed(d, remaining)
            axial_cmd = 0.0  # the dock point itself; the rate limit does the staging
        self._dock["commanded_speed"] = self._dock_speed
        goal = probe_dock.tip_goal(self.dock_world(), axial_cmd)
        step = self.cfg.approach.max_joint_step_rad if not final else 0.5 * self.cfg.approach.max_joint_step_rad
        self.track_probe(goal, probe_dock.decelerated(d, speed, remaining, d.z_decel_gain_hz) if speed > 0.0 else 1e-9, max_step=step)
        self._dock["min_clearance"] = min(float(self._dock.get("min_clearance", math.inf)), m["clearance"]) if m["insertion_depth"] > 0.0 else self._dock.get("min_clearance", math.inf)
        self._dock["max_lateral_inserting"] = max(float(self._dock.get("max_lateral_inserting", 0.0)), m["lateral"]) if m["insertion_depth"] > 0.0 else self._dock.get("max_lateral_inserting", 0.0)
        if not final and remaining <= d.insertion_zone_m:
            print(f"[DOCK] final insertion zone ({remaining*1000:.0f} mm remaining, depth {m['depth_distance']:.3f} m, "
                  f"wall clearance {m['clearance']*1000:.0f} mm)", flush=True)
            return self.goto(State.FINAL_INSERTION)
        if final and abs(m["axial"]) <= d.dock_axial_m:
            return self.goto(State.DOCK_READY)
        self._dock_stage_timeout(m)

    def step_dock_ready(self, m):
        """Every docking condition is re-checked here; only then is the joint created."""
        d = self.cfg.docking
        self.track_probe(probe_dock.tip_goal(self.dock_world(), 0.0), d.insertion_speed_mps,
                         max_step=0.5 * self.cfg.approach.max_joint_step_rad)
        valid, why = self.docking_tracking_valid()
        ready, bad = probe_dock.dock_ready(d, m, m["rel_speed"], bool(m["depth_ok"]), valid, m["clearance"])
        if ready:
            detail = (f"axial {m['axial']*1000:.2f} mm, radial {m['lateral']*1000:.2f} mm, axis {m['axis_deg']:.3f} deg, roll {m['roll_deg']:.3f} deg, "
                      f"depth {m['insertion_depth']:.3f} m inside the nozzle, relative velocity {m['rel_speed']*1000:.1f} mm/s, "
                      f"wall clearance {m['clearance']*1000:.0f} mm, depth reading {m['depth_distance']:.3f} m")
            print(f"[DOCK] DOCK_READY  t={self.sim_time:.2f} s | {detail}", flush=True)
            self.results.check("[DOCK4] Docking conditions", True, detail)
            sat_before = self.sat_frame()
            if self.orbit is not None:
                self._orbit["motion_before_dock"] = self.motion_snapshot()
            self.task.docking.dock()
            self._dock["docked"] = self.task.docking.is_docked
            self._dock["at_dock"] = {k: v for k, v in m.items() if isinstance(v, float)}
            self._dock["sat_at_dock"] = sat_before
            self._dock["rel_at_dock"] = self.probe_world().inv() @ self.sat_frame()
            self.results.check("[DOCK5] Docking FixedJoint created", bool(self.task.docking.is_docked),
                               "UsdPhysics.FixedJoint MEP <-> satellite anchored at the probe tip")
            self.q_hold = self.arm.joint_pos().clone()
            self.goto(State.DOCKED if self.task.docking.is_docked else State.DOCK_FAILED,
                      "" if self.task.docking.is_docked else "the docking joint was not created")
        elif self.state_time > 5.0:
            n = int(self._dock.get("ready_retries", 0))
            if n < d.dock_ready_retries:
                self._dock["ready_retries"] = n + 1
                self._dock_align_since = None
                print(f"[DOCK] conditions not met after 5 s ({'; '.join(bad)}): re-aligning ({n + 1}/{d.dock_ready_retries})", flush=True)
                return self.goto(State.XY_ALIGN, "; ".join(bad))
            self.results.check("[DOCK4] Docking conditions", False, "; ".join(bad))
            self.goto(State.DOCK_FAILED, "docking conditions not met: " + "; ".join(bad))

    def step_docked(self, m):
        self.dock_hold()
        # --orbit-return: straight on to the orbit phase (its own hold replaces DOCK_HOLDING)
        self.goto(State.ORBIT_TARGET_ACQUIRE if self.orbit_return_active else State.DOCK_HOLDING)

    def step_dock_holding(self, m):
        """Hold the docked state and watch for jumps / penetration / joint loss."""
        d = self.cfg.docking
        self.hold()
        h = self._dock
        rel_now = self.probe_world().inv() @ self.sat_frame()
        pe, ae = pose_errors(rel_now, h["rel_at_dock"])
        h["hold_max_drift_mm"] = max(float(h.get("hold_max_drift_mm", 0.0)), pe * 1000.0)
        h["hold_max_drift_deg"] = max(float(h.get("hold_max_drift_deg", 0.0)), ae)
        h["hold_joint_valid"] = bool(h.get("hold_joint_valid", True)) and self.task.docking.is_docked
        h["hold_min_clearance_mm"] = min(float(h.get("hold_min_clearance_mm", math.inf)), m["clearance"] * 1000.0)
        if self.state_time >= d.hold_duration_s:
            h["hold_duration_s"] = self.state_time
            self.results.check("[DOCK6] Docked state held", h["hold_joint_valid"] and h["hold_max_drift_mm"] < 5.0 and h["hold_max_drift_deg"] < 0.5,
                               f"{self.state_time:.1f} s: max MEP-satellite relative drift {h['hold_max_drift_mm']:.3f} mm / {h['hold_max_drift_deg']:.4f} deg, "
                               f"joint present the whole time: {h['hold_joint_valid']}")
            self.results.check("[DOCK7] No penetration while docked", h["hold_min_clearance_mm"] > 0.0,
                               f"min probe-to-wall clearance {h['hold_min_clearance_mm']:.0f} mm")
            self.goto(State.SUCCESS)

    ###########################################
    ### Phase 3: Client reference orbit return ###
    ###########################################

    @property
    def orbit_return_active(self) -> bool:
        """The return phase runs after the docking (`--orbit-return`); `--orbit-only` only checks the scene."""
        return self.orbit is not None and not self._observe_only and self.cfg.docking.enabled

    def client_ref_world(self) -> Frame:
        """The rigid Client point that has to reach the orbit (SAT_DOCK_POINT by default)."""
        return self.sat_frame() @ self.orbit.client_ref_in_sat

    def orbit_error(self) -> Tuple[float, np.ndarray, float]:
        """(distance [m], nearest orbit point, its theta) of the Client point."""
        q, theta, d = self.orbit.orbit.closest(self.client_ref_world().pos)
        return d, q, theta

    def orbit_joint_valid(self) -> bool:
        """Both FixedJoints hold: EE <-> MEP (capture) and MEP <-> Client (docking)."""
        return bool(self.task.docking.is_docked) and bool(self.capture.is_attached(0))

    def orbit_rel_drift(self) -> Tuple[float, float]:
        """MEP-Client relative pose drift since the docking joint was made: (mm, deg)."""
        rel_now = self.probe_world().inv() @ self.sat_frame()
        pe, ae = pose_errors(rel_now, self._dock["rel_at_dock"])
        return pe * 1000.0, ae

    def joint_limit_margin(self) -> float:
        """Smallest distance of an arm joint to its (soft) limits [rad]."""
        r = self.task._robot
        ids = self.arm.joint_ids
        lim = r.data.soft_joint_pos_limits[0, ids].cpu().numpy()
        return orbit_return.joint_margin(r.data.joint_pos[0, ids].cpu().numpy(), lim[:, 0], lim[:, 1])

    def client_commanded_velocity(self) -> np.ndarray:
        d = self.cfg.docking
        return np.asarray(d.satellite_drift_direction, dtype=float) * float(d.satellite_velocity_mps)

    def motion_snapshot(self) -> dict:
        """Linear / angular velocity (world) of both bodies, for the before / after comparisons."""
        sat, mep = self.task._satellite.data, self.mep.data
        return {"t": self.sim_time,
                "mep_v": mep.root_com_lin_vel_w[0].cpu().numpy().tolist(), "mep_w": mep.root_com_ang_vel_w[0].cpu().numpy().tolist(),
                "client_v": sat.root_com_lin_vel_w[0].cpu().numpy().tolist(), "client_w": sat.root_com_ang_vel_w[0].cpu().numpy().tolist()}

    def read_damping(self, path: str) -> Optional[Tuple[float, float]]:
        """(linear, angular) damping authored on the rigid body under `path`, or None."""
        from pxr import Usd

        root = self.stage.GetPrimAtPath(path)
        if not root.IsValid():
            return None
        for p in Usd.PrimRange(root):
            lin, ang = p.GetAttribute("physxRigidBody:linearDamping"), p.GetAttribute("physxRigidBody:angularDamping")
            if lin and lin.IsValid() and lin.HasValue():
                return float(lin.Get()), float(ang.Get()) if ang and ang.IsValid() and ang.HasValue() else math.nan
        return None

    def verify_orbit_prim(self) -> Tuple[bool, str]:
        """The red orbit prim exists, is visible, red, smooth and has no physics at all."""
        from pxr import Usd, UsdGeom

        t, oc = self.orbit, self.orbit_cfg
        root = self.stage.GetPrimAtPath(t.prim_path)
        if not root.IsValid():
            return False, f"{t.prim_path} does not exist"
        geoms, physics = [], []
        for p in Usd.PrimRange(root):
            if p.IsA(UsdGeom.Mesh) or p.IsA(UsdGeom.BasisCurves):
                geoms.append(p)
            applied = [str(a) for a in p.GetAppliedSchemas() if str(a).startswith(("Physics", "Physx", "PhysX"))]
            if applied:
                physics.append(f"{p.GetPath()}: {applied}")
        if not geoms:
            return False, f"{t.prim_path} has no geometry"
        if physics:
            return False, f"physics schemas on the orbit prim (it must be visual only): {physics}"
        colour = UsdGeom.Gprim(geoms[0]).GetDisplayColorAttr().Get()
        rgb = np.asarray(colour[0], dtype=float) if colour else np.full(3, math.nan)
        visible = UsdGeom.Imageable(root).ComputeVisibility() != UsdGeom.Tokens.invisible
        n_pts = len(UsdGeom.Mesh(geoms[0]).GetPointsAttr().Get()) if geoms[0].IsA(UsdGeom.Mesh) else 0
        ok = bool(np.allclose(rgb, oc.color_rgb, atol=1e-3)) and visible and n_pts >= oc.segments
        return ok, (f"{t.prim_path}: {len(geoms)} {geoms[0].GetTypeName()} prim(s), {n_pts} vertices for {oc.segments} segments, "
                    f"colour {np.round(rgb, 3).tolist()}, visible {visible}, no rigid body / collider / contact schema")

    def report_orbit_setup(self):
        """t = 0: log the orbit, the Client's start and verify the scene against the request."""
        c, oc, t, r = self.cfg, self.orbit_cfg, self.orbit, self.results
        orb = t.orbit
        ref0 = self.client_ref_world().pos
        prim_ok, prim_detail = self.verify_orbit_prim()
        off_ok, dist, off_detail = orbit_return.initial_offset_check(oc, orb, ref0)
        q_near, th_near, _ = orb.closest(ref0)
        direction = orbit_return.offset_direction(oc, orb, t.theta_nominal)
        mep_d, sat_d = self.read_damping(self.geo.mep_path), self.read_damping(self.geo.sat_prim_path)
        snap = self.motion_snapshot()
        mep_dv, mep_dw = orbit_return.free_flight_deviation(snap["mep_v"], c.mep.linear_velocity_w(), snap["mep_w"])
        sat_dv, sat_dw = orbit_return.free_flight_deviation(snap["client_v"], self.client_commanded_velocity(), snap["client_w"])
        print(f"[ORBIT] ---- reference orbit ({'circle' if orb.is_circle else 'ellipse'}, segments {oc.segments}, colour {oc.color_rgb}) ----", flush=True)
        print(f"[ORBIT] centre {np.round(orb.center, 4).tolist()} m ({'derived from the Client start' if t.center_derived else 'configured'}), "
              f"normal {np.round(orb.normal, 4).tolist()}, e1 {np.round(orb.e1, 4).tolist()}, "
              f"radius / semi-axes {orb.a:.3f} / {orb.b:.3f} m, line width {oc.line_width_m*1000:.0f} mm, prim {t.prim_path}", flush=True)
        print(f"[ORBIT] nominal orbit point (theta {math.degrees(t.theta_nominal):.2f} deg) {np.round(t.nominal_point, 4).tolist()}; "
              f"Client point '{oc.client_reference}' starts at {np.round(ref0, 4).tolist()} = nominal + {oc.client_initial_offset_m:.3f} m "
              f"along {np.round(direction, 4).tolist()} ({oc.client_offset_direction})", flush=True)
        tangent = orb.tangent_at(t.theta_nominal)
        drift_angles = {}
        for who, v_cmd in (("MEP", c.mep.linear_velocity_w()), ("Client", self.client_commanded_velocity())):
            n = float(np.linalg.norm(v_cmd))
            drift_angles[who] = None if n < 1e-12 else math.degrees(math.acos(min(1.0, abs(float(v_cmd @ tangent)) / n)))
        lo, hi = orb.theta_range
        print(f"[ORBIT] Earth (sky dome low_earth_orbit.exr): cap of {90.0 - orbit_return.EARTH_LIMB_DIP_DEG:.1f} deg around nadir -Z, limb {orbit_return.EARTH_LIMB_DIP_DEG} deg below the horizon; "
              f"the ring is concentric with it (normal {np.round(orb.normal, 3).tolist()}). Drawn arc: theta {math.degrees(lo):.1f} .. {math.degrees(hi):.1f} deg "
              f"({math.degrees(orb.arc_span):.0f} deg), end points {np.round(orb.point_at(lo), 2).tolist()} / {np.round(orb.point_at(hi), 2).tolist()}; "
              f"tangent at the nominal point {np.round(tangent, 4).tolist()}, drift-to-tangent angle {drift_angles}", flush=True)
        print(f"[ORBIT] Client start: {off_detail}; nearest orbit point {np.round(q_near, 4).tolist()}, out of the orbit plane {orb.out_of_plane(ref0)*1000:.1f} mm; "
              f"satellite moved by {np.round(t.placement_shift, 4).tolist()} m to get there", flush=True)
        print(f"[ORBIT] orbit clearance to the sensor rays (keep-out {oc.sensor_keepout_m} m): "
              f"{ {k: round(v, 3) for k, v in t.keepout_m.items()} }", flush=True)
        print(f"[ORBIT] MEP v {np.round(snap['mep_v'], 5).tolist()} m/s w {np.round(snap['mep_w'], 6).tolist()} rad/s | "
              f"Client v {np.round(snap['client_v'], 5).tolist()} m/s w {np.round(snap['client_w'], 6).tolist()} rad/s | "
              f"damping MEP {mep_d}, Client {sat_d} (linear, angular)", flush=True)
        r.metrics["orbit"] = {
            **orb.describe(), "prim_path": t.prim_path, "segments": oc.segments, "line_width_m": oc.line_width_m, "color_rgb": oc.color_rgb,
            "center_derived": t.center_derived, "client_reference": oc.client_reference,
            "theta_nominal_deg": math.degrees(t.theta_nominal), "nominal_point_w": t.nominal_point.tolist(),
            "client_start_w": ref0.tolist(), "client_start_before_shift_w": t.client_start_before_shift.tolist(),
            "satellite_placement_shift_m": t.placement_shift.tolist(), "client_offset_direction_w": direction.tolist(),
            "configured_offset_m": oc.client_initial_offset_m, "measured_start_distance_m": dist,
            "nearest_orbit_point_at_start_w": q_near.tolist(), "sensor_clearance_m": t.keepout_m,
            "damping_mep": mep_d, "damping_client": sat_d, "motion_start": snap,
            "earth_limb_dip_deg": orbit_return.EARTH_LIMB_DIP_DEG, "tangent_at_nominal_w": tangent.tolist(),
            "drift_to_tangent_deg": drift_angles,
        }
        r.check("[ORBIT0] Red reference orbit prim, visual only", prim_ok, prim_detail)
        r.check("[ORBIT0] Client starts off the orbit by the configured offset", off_ok, off_detail)
        r.check("[ORBIT0] Orbit clear of the depth ray and the AprilTag corridor", bool(t.keepout_ok),
                ", ".join(f"{k} {v:.2f} m" for k, v in t.keepout_m.items()) + f" (>= {oc.sensor_keepout_m} m)")
        tol_v, tol_w = c.drift.velocity_tolerance_mps, c.drift.angular_tolerance_rad_s
        r.check("[ORBIT0] MEP and Client: no rotation, commanded constant velocity",
                mep_dv <= tol_v and sat_dv <= tol_v and mep_dw <= tol_w and sat_dw <= tol_w,
                f"MEP |dv| {mep_dv*1000:.3f} mm/s |w| {mep_dw:.6f} rad/s; Client |dv| {sat_dv*1000:.3f} mm/s |w| {sat_dw:.6f} rad/s "
                f"(tolerance {tol_v*1000:.1f} mm/s, {tol_w:.4f} rad/s)")
        # The damping attribute must read back 0 (an unreadable one is left to the free-flight monitor)
        zero = all(d is None or (d[0] == 0.0 and (math.isnan(d[1]) or d[1] == 0.0)) for d in (mep_d, sat_d))
        r.check("[ORBIT0] MEP and Client drift along the orbit (tangent at the nominal point)",
                all(a is None or a <= 10.0 for a in drift_angles.values()),
                ", ".join(f"{k} {'at rest' if a is None else format(a, '.2f') + ' deg'}" for k, a in drift_angles.items()) + " off the tangent (<= 10 deg)")
        r.check("[ORBIT0] Linear and angular damping are 0 for both bodies", zero,
                f"MEP {mep_d}, Client {sat_d} (linear, angular; None = not readable, then only the free-flight monitor guards it)")

    def check_free_flight(self) -> Optional[str]:
        """A body that is free (MEP not yet captured, Client not yet docked) must keep the commanded
        velocity and must not turn: anything else (a push, damping, a collision) fails the run."""
        if self.orbit is None:
            return None
        drift = self.cfg.drift
        bodies = []
        if not self.capture.is_attached(0):
            bodies.append(("MEP", self.mep.data, self.cfg.mep.linear_velocity_w()))
        if not self.task.docking.is_docked:
            bodies.append(("Client", self.task._satellite.data, self.client_commanded_velocity()))
        for name, d, cmd in bodies:
            v, w = d.root_com_lin_vel_w[0].cpu().numpy(), d.root_com_ang_vel_w[0].cpu().numpy()
            dv, dw = orbit_return.free_flight_deviation(v, cmd, w)
            st = self._free_flight.setdefault(name, {"samples": 0, "max_dv_mps": 0.0, "max_w_rad_s": 0.0})
            st["samples"] += 1
            st["max_dv_mps"], st["max_w_rad_s"] = max(st["max_dv_mps"], dv), max(st["max_w_rad_s"], dw)
            ok, why = orbit_return.free_flight_ok(drift, v, cmd, w)
            if not ok:
                return f"{name} free flight violated at t={self.sim_time:.2f} s: {why} (v {np.round(v, 5).tolist()}, commanded {np.round(cmd, 5).tolist()}, w {np.round(w, 5).tolist()})"
        return None

    def orbit_guard(self) -> bool:
        """Checks common to every orbit-return state; False once it has switched to ORBIT_FAILED."""
        oc, h = self.orbit_cfg, self._orbit
        if not self.orbit_joint_valid():
            self.goto(State.ORBIT_FAILED, f"FixedJoint lost (docking joint {self.task.docking.is_docked}, capture joint {self.capture.is_attached(0)})")
            return False
        ref = self.client_ref_world().pos
        prev = h.get("prev_ref")
        jump = float(np.linalg.norm(ref - prev)) if prev is not None else 0.0
        h["prev_ref"] = ref.copy()
        h["max_step_jump_m"] = max(float(h.get("max_step_jump_m", 0.0)), jump)
        if jump > oc.max_step_jump_m:
            self.goto(State.ORBIT_FAILED, f"the Client point moved {jump*1000:.1f} mm in one control step (> {oc.max_step_jump_m*1000:.1f} mm): "
                                          "that is a jump, not a controlled motion")
            return False
        margin = self.joint_limit_margin()
        h["min_joint_margin_rad"] = min(float(h.get("min_joint_margin_rad", math.inf)), margin)
        if margin < oc.joint_limit_margin_rad:
            self.goto(State.ORBIT_FAILED, f"an arm joint is {margin:.4f} rad from its limit (< {oc.joint_limit_margin_rad} rad): the target is at the edge of the workspace")
            return False
        dp, da = self.orbit_rel_drift()
        h["max_drift_mm"], h["max_drift_deg"] = max(float(h.get("max_drift_mm", 0.0)), dp), max(float(h.get("max_drift_deg", 0.0)), da)
        w = float(torch.norm(self.task._satellite.data.root_com_ang_vel_w[0]))
        h["max_client_w_rad_s"] = max(float(h.get("max_client_w_rad_s", 0.0)), w)
        h["min_error_m"] = min(float(h.get("min_error_m", math.inf)), self.orbit_error()[0])
        return True

    def latch_orbit_target(self, target: "orbit_return.OrbitTarget"):
        """Arm goal for a Client-point target: the Client is rigid with the probe tip (the IK tool)
        and the orientation is held, so the goal is the tool pose moved by (target - Client point).
        Latched (not re-derived from the swinging arm every step)."""
        tool, ref = self.arm.tool_pose(), self.client_ref_world()
        self._orbit["target"] = {"theta_deg": math.degrees(target.theta), "point": target.point.copy(),
                                 "distance_at_selection_m": target.distance_m, "client_point": ref.pos.copy()}
        self._orbit_goal = Frame(tool.pos + (target.point - ref.pos), tool.rot.copy())
        self._orbit_tangent = self.orbit.orbit.tangent_at(target.theta)
        self._dock_ref = tool
        self._dock_speed = 0.0
        self.q_hold = None

    def step_orbit_target_acquire(self, m):
        """Pick the orbit point the Client point is carried to: the nearest one the arm can reach."""
        self.dock_hold()
        if not self.orbit_guard() or self.state_time < self.cfg.docking.settle_time_s:
            return
        oc, h = self.orbit_cfg, self._orbit
        ref, ee, base = self.client_ref_world(), self.ee_pose(), self.arm.base.pos
        h["motion_after_dock"] = self.motion_snapshot()
        h["client_at_dock_w"] = ref.pos.tolist()
        h["error_at_dock_m"] = self.orbit_error()[0]

        def feasible(p):
            # The arm's reach limits the EE contact point that holds the MEP (the probe tip / Client
            # point are metres further out, rigid with it): everything moves by (p - Client point)
            return orbit_return.reach_feasible(ee.pos + (p - ref.pos), base, oc.arm_reach_min_m, oc.arm_reach_max_m)

        target, tried = orbit_return.select_target(self.orbit.orbit, ref.pos, feasible, oc.max_target_candidates)
        h["candidates_tried"] = tried[:8]
        if target is None:
            self.results.check("[ORBIT1] Orbit target acquired", False, f"no reachable orbit point among {len(tried)} candidates: {tried[:3]}")
            return self.goto(State.ORBIT_FAILED, f"no reachable orbit point among {len(tried)} candidates (first: {tried[0]['why']})")
        self.latch_orbit_target(target)
        goal_dist = float(np.linalg.norm(target.point - ref.pos))
        h["transfer_distance_m"] = goal_dist
        self.results.check("[ORBIT1] Orbit target acquired", True,
                           f"orbit centre {np.round(self.orbit.orbit.center, 3).tolist()}, normal {np.round(self.orbit.orbit.normal, 3).tolist()}, "
                           f"target {np.round(target.point, 3).tolist()} (theta {math.degrees(target.theta):.1f} deg), Client point {np.round(ref.pos, 3).tolist()} "
                           f"is {target.distance_m*1000:.0f} mm from it; EE goal {tried[-1]['why']}; "
                           f"{len([x for x in tried if not x['feasible']])} nearer candidate(s) rejected")
        print(f"[ORBIT] target acquired: {np.round(target.point, 4).tolist()} (theta {math.degrees(target.theta):.2f} deg), "
              f"transfer {goal_dist:.3f} m at <= {oc.transfer_speed_mps} m/s, probe-tip goal {np.round(self._orbit_goal.pos, 4).tolist()}", flush=True)
        self.frame_orbit_view()
        self.goto(State.ORBIT_TRANSFER)

    def step_orbit_transfer(self, m):
        """Carry the joined MEP + Client with the arm: continuous IK tracking (rate-limited, decelerating
        towards the goal). Nothing is written to a body pose and no joint is recreated."""
        if not self.orbit_guard():
            return
        oc, d, h = self.orbit_cfg, self.cfg.docking, self._orbit
        # Only the distance to the orbit matters, not the position along it: the goal follows the joined bodies
        # along the orbit tangent, so the arm does not have to stop the (very heavy) Client's sideways drift
        goal = self._orbit_goal
        tool = self.arm.tool_pose()
        if self._orbit_tangent is not None:
            goal = Frame(goal.pos + self._orbit_tangent * float((tool.pos - goal.pos) @ self._orbit_tangent), goal.rot)
        pe, _ = pose_errors(tool, goal)
        speed = probe_dock.decelerated(d, oc.transfer_speed_mps, pe, oc.transfer_decel_gain_hz)
        self._dock["commanded_speed"] = speed
        self.track_probe(goal, speed, max_step=0.5 * self.cfg.approach.max_joint_step_rad, follow_satellite=False, hold_tol_m=0.01,
                         lag_limit_m=oc.transfer_lag_limit_m, accel_mps2=oc.transfer_accel_mps2)
        if pe < 0.02 and self.tip_settled():
            if "transfer_checked" not in h:
                h["transfer_checked"] = True
                self.results.check("[ORBIT2] Transfer with both FixedJoints kept, no jump", h["max_step_jump_m"] <= oc.max_step_jump_m,
                                   f"max Client-point step {h['max_step_jump_m']*1000:.3f} mm per control step (<= {oc.max_step_jump_m*1000:.1f}), "
                                   f"joints valid throughout, min arm joint margin {h['min_joint_margin_rad']:.3f} rad, "
                                   f"MEP-Client drift {h['max_drift_mm']:.3f} mm / {h['max_drift_deg']:.4f} deg")
            h["arrival_since"] = None
            print(f"[ORBIT] transfer finished after {self.state_time:.1f} s: Client point {self.orbit_error()[0]*1000:.1f} mm from the orbit", flush=True)
            self.goto(State.ORBIT_ARRIVAL_CHECK)
        elif self.state_time > oc.transfer_timeout_s:
            self.goto(State.ORBIT_FAILED, f"transfer not finished within {oc.transfer_timeout_s:.0f} s (tool {pe*1000:.0f} mm from its goal, "
                                          f"orbit error {self.orbit_error()[0]*1000:.0f} mm)")

    def step_orbit_arrival_check(self, m):
        """All arrival conditions together for `arrival_hold_s`; a residual position error (after the swing has
        died out) is corrected by a new, smaller transfer to the nearest orbit point."""
        self.hold()
        if not self.orbit_guard():
            return
        oc, h = self.orbit_cfg, self._orbit
        err, nearest, theta = self.orbit_error()
        dp, da = self.orbit_rel_drift()
        w = float(torch.norm(self.task._satellite.data.root_com_ang_vel_w[0]))
        ok, bad = orbit_return.arrival_ok(oc, err, self.orbit_joint_valid(), dp, da, w)
        if ok:
            if h.get("arrival_since") is None:
                h["arrival_since"] = self.sim_time
            if self.sim_time - h["arrival_since"] >= oc.arrival_hold_s:
                h["arrival"] = {"time_s": self.sim_time, "error_m": err, "drift_mm": dp, "drift_deg": da, "client_w_rad_s": w, "retargets": int(h.get("retargets", 0))}
                self.results.check("[ORBIT3] Client point on the orbit", True,
                                   f"{err*1000:.1f} mm from the orbit (<= {oc.arrival_tolerance_m*1000:.0f} mm) at t={self.sim_time:.1f} s, "
                                   f"MEP-Client drift {dp:.3f} mm / {da:.4f} deg, Client |w| {w:.5f} rad/s, {int(h.get('retargets', 0))} retarget(s)")
                print(f"[ORBIT] arrived: Client point {err*1000:.1f} mm from the orbit; holding {oc.hold_duration_s:.0f} s", flush=True)
                self.q_hold = None
                return self.goto(State.ORBIT_HOLDING)
            return
        h["arrival_since"] = None
        if err > oc.arrival_tolerance_m and self.tip_settled():
            n = int(h.get("retargets", 0))
            if n >= oc.max_retargets:
                self.results.check("[ORBIT3] Client point on the orbit", False, f"{err*1000:.1f} mm from the orbit after {n} corrections: {'; '.join(bad)}")
                return self.goto(State.ORBIT_FAILED, f"still {err*1000:.1f} mm from the orbit after {n} corrections")
            h["retargets"] = n + 1
            print(f"[ORBIT] settled {err*1000:.1f} mm from the orbit (> {oc.arrival_tolerance_m*1000:.0f} mm): correction {n + 1}/{oc.max_retargets}", flush=True)
            self.latch_orbit_target(orbit_return.OrbitTarget(theta, nearest, err))
            return self.goto(State.ORBIT_TRANSFER, "correction")
        if self.state_time > oc.arrival_timeout_s:
            self.results.check("[ORBIT3] Client point on the orbit", False, "; ".join(bad))
            self.goto(State.ORBIT_FAILED, "arrival conditions not met: " + "; ".join(bad))

    def step_orbit_holding(self, m):
        """Hold the joined bodies on the orbit and record the error, the drift and the Client rotation."""
        self.hold()
        if not self.orbit_guard():
            return
        oc, h = self.orbit_cfg, self._orbit
        hs = h.setdefault("hold", {"max_error_m": 0.0, "joint_valid": True})
        err = self.orbit_error()[0]
        hs["max_error_m"] = max(hs["max_error_m"], err)
        dp, da = self.orbit_rel_drift()
        hs["max_drift_mm"], hs["max_drift_deg"] = max(float(hs.get("max_drift_mm", 0.0)), dp), max(float(hs.get("max_drift_deg", 0.0)), da)
        hs["max_client_w_rad_s"] = max(float(hs.get("max_client_w_rad_s", 0.0)), float(torch.norm(self.task._satellite.data.root_com_ang_vel_w[0])))
        hs["joint_valid"] = bool(hs["joint_valid"]) and self.orbit_joint_valid()
        if self.state_time >= oc.hold_duration_s:
            hs["duration_s"] = self.state_time
            hs["final_error_m"] = err
            ok, bad = orbit_return.arrival_ok(oc, hs["max_error_m"], hs["joint_valid"], hs["max_drift_mm"], hs["max_drift_deg"], hs["max_client_w_rad_s"])
            h["motion_end"] = self.motion_snapshot()
            self.results.check("[ORBIT4] Held on the orbit", ok,
                               f"{self.state_time:.1f} s: max orbit error {hs['max_error_m']*1000:.1f} mm (<= {oc.arrival_tolerance_m*1000:.0f}), "
                               f"max MEP-Client drift {hs['max_drift_mm']:.3f} mm / {hs['max_drift_deg']:.4f} deg, max Client |w| {hs['max_client_w_rad_s']:.5f} rad/s, "
                               f"FixedJoints valid the whole time: {hs['joint_valid']}" + ("" if ok else "; FAILED: " + "; ".join(bad)))
            self.goto(State.SUCCESS if ok else State.ORBIT_FAILED, "" if ok else "; ".join(bad))

    ## Visuals / logging
    def frame_orbit_view(self):
        """GUI: look at the Client point and its orbit target from above-side."""
        if self.headless or self.orbit is None:
            return
        try:
            tgt = self._orbit["target"]["point"]
            mid = 0.5 * (tgt + self.client_ref_world().pos)
            o = self.orbit.orbit
            eye = mid + 5.0 * o.normal - 7.0 * o.e2
            self.sim.set_camera_view(eye=tuple(eye.tolist()), target=tuple(mid.tolist()))
            print("[VIS] viewport moved to the orbit target", flush=True)
        except Exception as e:
            print(f"[VIS] could not move the viewport: {e}", flush=True)

    def save_orbit_view_image(self):
        """`--orbit-only`: save the image of `cam_orbit_view` (at the GUI start view of the orbit) as
        `<tag>_orbit_view.png`, to check the red arc against the Earth of the sky dome."""
        try:
            import cv2

            from .vision_task import ORBIT_VIEW_CAMERA

            cam = self.scene[ORBIT_VIEW_CAMERA]
            img = np.ascontiguousarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
            path = self.out_dir / f"{self.label}_orbit_view.png"
            cv2.imwrite(str(path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            eye, target = self.orbit.view()
            print(f"[ORBIT] view image (eye {np.round(eye, 2).tolist()} -> {np.round(target, 2).tolist()}): {path}", flush=True)
            self.results.metrics.setdefault("orbit", {})["view_image"] = str(path)
        except Exception as e:
            print(f"[ORBIT] view image not saved: {e}", flush=True)

    def update_orbit_visuals(self):
        """Debug draw in the orbit states: Client point (cyan), orbit target (white) and the line between them."""
        ref = self.client_ref_world().pos
        tgt = self._orbit.get("target")
        aim = tgt["point"] if tgt else self.orbit_error()[1]
        self.vis.set_scene([], None, None, None, None, ref, aim, axes=[(self.dock_world(), 0.8, 5, 0.6)])
        self.vis.show()

    def orbit_status(self):
        """HUD line for the orbit states."""
        err, nearest, _ = self.orbit_error()
        tgt = self._orbit.get("target")
        dp, da = self.orbit_rel_drift()
        w = float(torch.norm(self.task._satellite.data.root_com_ang_vel_w[0]))
        print(f"[ORBIT] t={self.sim_time:6.1f}s {self.state.value:<20} orbit error {err*1000:8.1f} mm | "
              f"target {np.round(tgt['point'], 3).tolist() if tgt else '--'} | Client w {w:.5f} rad/s | MEP-Client drift {dp:.3f} mm {da:.4f} deg | "
              f"joint margin {self.joint_limit_margin():.3f} rad | joints {'ok' if self.orbit_joint_valid() else 'LOST'} | "
              f"tool lag {float(self._dock.get('tool_lag_m', math.nan))*1000:.0f} mm, ref->goal "
              f"{(float(np.linalg.norm(self._orbit_goal.pos - self._dock_ref.pos))*1000 if self._orbit_goal is not None and self._dock_ref is not None else math.nan):.0f} mm | "
              f"wall-clock {time.time() - self.wall_start:.0f} s", flush=True)

    def print_orbit_failure_context(self):
        """On every failure: what the orbit-return needs to be debugged from the log alone."""
        try:
            ref = self.client_ref_world().pos
            err, nearest, _ = self.orbit_error()
            tgt = self._orbit.get("target")
            sat, mep = self.task._satellite.data, self.mep.data
            print(f"[ORBIT] failure context: state before {self.state.value} (t={self.sim_time:.2f} s) | Client point {np.round(ref, 4).tolist()} | "
                  f"orbit error {err*1000:.1f} mm (nearest {np.round(nearest, 4).tolist()}) | target {np.round(tgt['point'], 4).tolist() if tgt else None} | "
                  f"docking joint {bool(self.task.docking.is_docked)}, capture joint {bool(self.capture.is_attached(0))} | "
                  f"Client |w| {float(torch.norm(sat.root_com_ang_vel_w[0])):.5f} rad/s, MEP |w| {float(torch.norm(mep.root_com_ang_vel_w[0])):.5f} rad/s | "
                  f"Client v {np.round(sat.root_com_lin_vel_w[0].cpu().numpy(), 5).tolist()} m/s", flush=True)
        except Exception as e:  # diagnostics must never hide the failure itself
            print(f"[ORBIT] failure context unavailable: {e}", flush=True)

    def log_orbit_row(self):
        ref = self.client_ref_world().pos
        err, nearest, _ = self.orbit_error()
        tgt = self._orbit.get("target")
        sat, mep = self.task._satellite.data, self.mep.data
        docked = bool(self.task.docking.is_docked) and "rel_at_dock" in self._dock
        dp, da = self.orbit_rel_drift() if docked else (math.nan, math.nan)
        row = {"run_id": self.label, "timestamp": round(self.sim_time, 4), "state": self.state.value,
               "orbit_error_m": err, "orbit_out_of_plane_m": self.orbit.orbit.out_of_plane(ref),
               "mep_client_drift_mm": dp, "mep_client_drift_deg": da,
               "docking_joint_valid": int(self.task.docking.is_docked), "capture_joint_valid": int(self.capture.is_attached(0)),
               "arm_joint_margin_rad": self.joint_limit_margin()}
        for name, p in (("client_ref", ref), ("orbit_nearest", nearest), ("target", tgt["point"] if tgt else np.full(3, math.nan))):
            row.update({f"{name}_{a}": float(v) for a, v in zip("xyz", p)})
        for name, body in (("client", sat), ("mep", mep)):
            row.update({f"{name}_v{a}": float(v) for a, v in zip("xyz", body.root_com_lin_vel_w[0].cpu().numpy())})
            row.update({f"{name}_w{a}": float(v) for a, v in zip("xyz", body.root_com_ang_vel_w[0].cpu().numpy())})
        self._orbit_rows.append(row)
        if self._orbit_csv is not None:
            self._orbit_csv.writerow({k: row.get(k, "") for k in ORBIT_CSV_COLUMNS})

    ## Depth calibration / logging
    def set_dock_ring_color(self, rgb):
        """Colour of the nozzle rim ring (state colour), only when the colour changes."""
        path = getattr(self.task, "dock_ring_path", None)
        if path is None or getattr(self, "_ring_rgb", None) == tuple(rgb):
            return
        self._ring_rgb = tuple(rgb)
        from pxr import Gf, Sdf, Usd, UsdShade

        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(path)):
            if prim.IsA(UsdShade.Shader):
                sh = UsdShade.Shader(prim)
                for name in ("diffuseColor", "emissiveColor"):
                    sh.CreateInput(name, Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))

    def frame_docking_view(self, close: bool):
        """GUI: point the viewport at the docking area. The startup view is aimed at the
        capture point (x ~ 5 m) while the docking happens around x ~ 15 m, y ~ 3 m, so
        without this every docking marker is outside the picture."""
        if self.headless:
            return
        try:
            probe, dock = self.probe_world(), self.dock_world()
            axis = dock.rot[:, 2]
            up = np.array([0.0, 0.0, 1.0])
            side = np.cross(axis, up)
            side = side / max(1e-9, float(np.linalg.norm(side)))
            if close:  # the last metres: nozzle and probe tip
                target = dock.pos - 0.6 * axis
                eye = target + 5.0 * side + 2.0 * up - 1.5 * axis
            else:      # the whole transport: from the tip to the nozzle
                target = 0.5 * (probe.pos + dock.pos)
                eye = target + (0.9 * float(np.linalg.norm(probe.pos - dock.pos)) + 4.0) * side + 3.0 * up
            self.sim.set_camera_view(eye=tuple(eye.tolist()), target=tuple(target.tolist()))
            print(f"[VIS] viewport moved to the docking area ({'close' if close else 'wide'} view)", flush=True)
        except Exception as e:
            print(f"[VIS] could not move the viewport: {e}", flush=True)

    def update_dock_visuals(self, m):
        """Debug draw for the docking phase: probe tip, SAT_DOCK_POINT, pre-dock point,
        both coordinate frames and the probe -> dock vector, colour-coded by state."""
        probe, dock = self.probe_world(), self.dock_world()
        pre = probe_dock.tip_goal(dock, self._pre_dock_axial())
        aligned = probe_dock.alignment_ok(self.cfg.docking, m)[0]
        if self.task.docking.is_docked:
            col = (0.1, 1.0, 0.2, 1.0)        # DOCKED
        elif self.state in (State.Z_APPROACH, State.FINAL_INSERTION, State.DOCK_READY):
            col = (1.0, 0.55, 0.0, 1.0)       # INSERTING
        elif aligned:
            col = (1.0, 1.0, 0.2, 1.0)        # READY
        else:
            col = (1.0, 0.2, 0.2, 1.0)        # NOT ALIGNED
        self.set_dock_ring_color(col[:3])
        axes = [(probe, 0.8, 5, 1.0), (dock, 0.8, 5, 0.6), (pre, 0.4, 2, 0.4)]
        vectors = [(probe.pos, dock.pos, col, 3)]
        self.vis.set_scene([], pre.pos, probe.pos, dock.pos, None, self.ee_pose().pos, None,
                           axes=axes, vectors=vectors)
        self.vis.show()

    def dock_status(self, m):
        """HUD line: state, the dock-frame errors, both distances and the dock flags."""
        print(f"[DOCK] t={self.sim_time:6.1f}s {self.state.value:<20} "
              f"e_xyz {m['e_x']*1000:7.1f} {m['e_y']*1000:7.1f} {m['e_z']*1000:8.1f} mm | "
              f"rpy {m['roll_rpy_deg']:6.2f} {m['pitch_rpy_deg']:6.2f} {m['yaw_rpy_deg']:6.2f} deg | "
              f"axis {m['axis_deg']:5.2f} deg | geo {m['geometry_distance']:.3f} m depth {m['depth_distance']:.3f} m "
              f"({int(m['depth_pixels'])} px) | rel.v {m['rel_speed']*1000:5.1f} mm/s | "
              f"align {'Y' if probe_dock.alignment_ok(self.cfg.docking, m)[0] else 'n'} "
              f"depth {'Y' if m['depth_ok'] else 'n'} docked {'Y' if self.task.docking.is_docked else 'n'} | "
              f"wall {('--' if not math.isfinite(m['clearance']) else format(m['clearance']*1000, '.0f')):>5} mm | wall-clock {time.time() - self.wall_start:.0f} s", flush=True)

    def _calibrate_depth(self, m):
        """Measure the depth offset once, where the geometric distance is known.

        `docking.depth_surface_offset_m` assumes the ray lands on the nozzle back plate,
        but that plate is a collision-only prim, so the real surface is the thruster mesh
        behind it. Both values are logged so the assumption is visible.
        """
        if not math.isfinite(m["depth_raw"]) or m["depth_pixels"] < 4:
            return
        measured = m["depth_raw"] - self._cam_to_tip - m["geometry_distance"]
        # Fail-safe: a ray that missed the nozzle gives a nonsense offset. The surface
        # can only be between the dock point and the far end of the thruster.
        if not -0.5 <= measured <= 3.0:
            if not self._calib_rejected:
                self._calib_rejected = True
                print(f"[DOCK] depth calibration rejected: implied surface offset {measured:.2f} m is outside "
                      f"[-0.5, 3.0] m (raw {m['depth_raw']:.3f} m, geometry {m['geometry_distance']:.3f} m); "
                      f"the ray is not landing inside the nozzle", flush=True)
            return
        self._depth_offset = float(measured)
        self._depth_calibrated_t = self.sim_time
        cfgd = self.cfg.docking.depth_surface_offset_m
        self.results.metrics.setdefault("docking", {})["depth_calibration"] = {
            "measured_surface_offset_m": self._depth_offset,
            "configured_surface_offset_m": cfgd,
            "backstop_gap_m": float(self.task.cfg.docking.backstop_gap),
            "geometry_distance_m": m["geometry_distance"],
            "depth_raw_m": m["depth_raw"],
            "camera_to_tip_m": self._cam_to_tip,
        }
        print(f"[DOCK] depth calibration at the pre-dock pose: raw {m['depth_raw']:.3f} m, geometry {m['geometry_distance']:.3f} m "
              f"-> surface offset {self._depth_offset:.3f} m (configured {cfgd:.3f} m, back plate at {self.task.cfg.docking.backstop_gap:.3f} m)", flush=True)

    def log_dock_row(self, m):
        row = {
            "run_id": self.label, "timestamp": round(self.sim_time, 4), "state": self.state.value,
            "commanded_speed_mps": float(self._dock.get("commanded_speed", 0.0)),
            "alignment_valid": int(probe_dock.alignment_ok(self.cfg.docking, m)[0]),
            "depth_valid": int(m["depth_ok"]), "dock_ready": int(self.state == State.DOCK_READY),
            "dock_success": int(self.task.docking.is_docked),
        }
        probe, dock = self.probe_world(), self.dock_world()
        for name, f in (("probe_tip", probe), ("sat_dock", dock)):
            row.update({f"{name}_{a}": float(v) for a, v in zip("xyz", f.pos)})
            row.update({f"{name}_q{a}": float(v) for a, v in zip("wxyz", f.quat)})
        row.update({
            "relative_x": m["e_x"], "relative_y": m["e_y"], "relative_z": m["e_z"],
            "roll_error_deg": m["roll_rpy_deg"], "pitch_error_deg": m["pitch_rpy_deg"], "yaw_error_deg": m["yaw_rpy_deg"],
            "orientation_error_deg": m["orientation_deg"], "axis_error_deg": m["axis_deg"],
            "roll_about_axis_deg": m["roll_deg"], "lateral_error_m": m["lateral"],
            "geometry_distance_m": m["geometry_distance"], "depth_distance_m": m["depth_distance"],
            "depth_raw_m": m["depth_raw"], "depth_pixels": m["depth_pixels"],
            "insertion_depth_m": m["insertion_depth"], "wall_clearance_m": m["clearance"],
            "relative_vx": m["rel_vx"], "relative_vy": m["rel_vy"], "relative_vz": m["rel_vz"],
            "relative_speed_mps": m["rel_speed"],
        })
        self._dock_rows.append(row)
        if self._dock_csv is not None:
            self._dock_csv.writerow({k: row.get(k, "") for k in probe_dock.CSV_COLUMNS})

    def save_probe_rgb(self):
        """One `cam_probe` RGB frame (the operator's view of the approach)."""
        pc = self.cfg.probe_camera
        if self._probe_cam is None or pc.rgb_every_sec <= 0.0:
            return
        out = self._probe_cam.data.output
        if "rgb" not in out:
            return
        import cv2

        img = np.ascontiguousarray(out["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8))
        path = self._probe_dir / f"{self.sim_time:07.2f}s_{self.state.value}.png"
        cv2.imwrite(str(path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        self._probe_frames.append(path)

    def _goal_errors(self, goal: Frame) -> Tuple[float, float, float]:
        ee = self.ee_pose()
        n = goal.rot[:, 2]
        d = ee.pos - goal.pos
        axial = float(d @ n)
        return float(np.linalg.norm(d - axial * n)), math.degrees(rotation_angle(ee.rot, goal.rot)), axial

    def _settled(self, lat_tol: float, ang_tol: float, axial_tol: float, hold_s: float) -> bool:
        if self.goal is None:
            return False
        lat, ang, axial = self._goal_errors(self.goal)
        if lat < lat_tol and ang < ang_tol and abs(axial) < axial_tol:
            if self._settled_since is None:
                self._settled_since = self.sim_time
            return self.sim_time - self._settled_since >= hold_s
        self._settled_since = None
        return False

    ###############
    ### Logging ###
    ###############

    def log_row(self):
        gt = self.gt_cylinder()
        est = self.estimate_now()
        pred = self.predicted()
        ee = self.ee_pose()
        # MEP velocity at the EE contact point, as the GT columns (= v in translation_only)
        v_est, w_twist = self.est_mep_twist_at(ee.pos)
        # translation_only: Phase 1 monitoring rate (not used by the controller)
        w_est = w_twist if self.six_dof else self.predictor.angular_velocity()
        if w_est is None:
            w_est = np.full(3, math.nan)
        v_gt, w_gt = self.gt_mep_velocity_at(ee.pos)
        m = self.capture_metrics(est, v_est, w_twist)
        m_gt = self.capture_metrics(gt, v_gt, w_gt)
        lv = self.last_vision
        ee_w = self.ee_angular_velocity()
        tracking_ok, _ = self.tracking_ok()

        def quat(f: Optional[Frame], prefix: str) -> dict:
            q = f.quat if f is not None else (math.nan,) * 4
            return {f"{prefix}_q{k}": v for k, v in zip("wxyz", q)}
        base = self.base_frame()
        est_b = (base.inv() @ est).pos if est is not None else None
        g_rpy = rotmat_to_rpy_deg(gt.rot)
        e_rpy = rotmat_to_rpy_deg(est.rot) if est is not None else (math.nan,) * 3
        pe, ae = pose_errors(gt, est) if est is not None else (math.nan, math.nan)
        rpe, rae = pose_errors(gt, lv.t_w_y) if lv is not None and lv.valid else (math.nan, math.nan)
        drift = self.relative_drift() if self._rel_at_capture is not None else (math.nan, math.nan)
        row = {
            "timestamp": round(self.sim_time, 4),
            "gt_x": gt.pos[0], "gt_y": gt.pos[1], "gt_z": gt.pos[2],
            "gt_roll": g_rpy[0], "gt_pitch": g_rpy[1], "gt_yaw": g_rpy[2],
            "est_x": est.pos[0] if est else math.nan, "est_y": est.pos[1] if est else math.nan, "est_z": est.pos[2] if est else math.nan,
            "est_roll": e_rpy[0], "est_pitch": e_rpy[1], "est_yaw": e_rpy[2],
            # error of the system estimate (est_*) vs the GT at the same time
            "position_error_mm": pe * 1000.0, "angle_error_deg": ae,
            "pred_x": pred.pos[0] if pred else math.nan, "pred_y": pred.pos[1] if pred else math.nan, "pred_z": pred.pos[2] if pred else math.nan,
            "ee_x": ee.pos[0], "ee_y": ee.pos[1], "ee_z": ee.pos[2],
            "ee_target_x": self.goal.pos[0] if self.goal else math.nan,
            "ee_target_y": self.goal.pos[1] if self.goal else math.nan,
            "ee_target_z": self.goal.pos[2] if self.goal else math.nan,
            "ee_target_distance_m": float(np.linalg.norm(ee.pos - self.goal.pos)) if self.goal else math.nan,
            "relative_velocity_mps": m["rel_vel"],
            "mep_linear_velocity_x": v_est[0] if v_est is not None else math.nan,
            "mep_linear_velocity_y": v_est[1] if v_est is not None else math.nan,
            "mep_linear_velocity_z": v_est[2] if v_est is not None else math.nan,
            "mep_angular_velocity_x": w_est[0], "mep_angular_velocity_y": w_est[1], "mep_angular_velocity_z": w_est[2],
            "num_detected_tags": len(lv.detections) if lv else 0,
            "capture_state": self.state.value,
            "capture_success": int(self.capture.is_attached(0)),
            "total_elapsed_time": round(time.time() - self.wall_start, 3),
            "scenario": self.label,
            "est_age_s": self.estimate_age(),
            "reproj_rms_px": lv.reproj_rms if lv else math.nan,
            "pnp_inliers": lv.inliers if lv else 0,
            "raw_position_error_mm": rpe * 1000.0, "raw_angle_error_deg": rae,
            "est_base_x": est_b[0] if est_b is not None else math.nan,
            "est_base_y": est_b[1] if est_b is not None else math.nan,
            "est_base_z": est_b[2] if est_b is not None else math.nan,
            "gt_mep_linear_velocity_x": v_gt[0], "gt_mep_linear_velocity_y": v_gt[1], "gt_mep_linear_velocity_z": v_gt[2],
            "gt_mep_angular_velocity_deg_s": math.degrees(float(np.linalg.norm(w_gt))),
            "gt_relative_velocity_mps": m_gt["rel_vel"],
            "ee_to_cylinder_distance_m": m_gt["distance"],
            "ee_lateral_error_m": m_gt["lateral"],
            "ee_gap_m": m_gt["gap"],
            "ee_normal_angle_deg": m_gt["angle"],
            "standoff_cmd_m": self.standoff,
            "fixed_joint_valid": int(self.capture.is_attached(0)),
            "mep_ee_relative_drift_mm": drift[0] * 1000.0,
            "mep_ee_relative_drift_deg": drift[1],
            "motion_mode": self.cfg.mep.motion_mode,
            "prediction_horizon_s": self.cfg.prediction.horizon_sec,
            **quat(gt, "gt"), **quat(est, "est"), **quat(pred, "pred"), **quat(ee, "ee"), **quat(self.goal, "ee_target"),
            "gt_mep_angular_velocity_x": w_gt[0], "gt_mep_angular_velocity_y": w_gt[1], "gt_mep_angular_velocity_z": w_gt[2],
            "ee_angular_velocity_x": ee_w[0], "ee_angular_velocity_y": ee_w[1], "ee_angular_velocity_z": ee_w[2],
            "est_ee_to_cylinder_distance_m": m["distance"],
            "est_ee_lateral_error_m": m["lateral"],
            "est_ee_normal_angle_deg": m["angle"],
            "est_ee_orientation_error_deg": m["orientation"],
            "est_relative_angular_velocity_rad_s": m["rel_ang_vel"],
            "gt_ee_orientation_error_deg": m_gt["orientation"],
            "gt_relative_angular_velocity_rad_s": m_gt["rel_ang_vel"],
            "pnp_valid": int(bool(lv is not None and lv.valid)),
            "tracking_valid": int(tracking_ok),
        }
        row = {k: (round(float(v), 6) if isinstance(v, (float, np.floating)) else v) for k, v in row.items()}
        if self._csv is not None:
            self._csv.writerow(row)
        self.rows.append(row)

    def update_visuals(self):
        lv = self.last_valid
        tags_w: List[np.ndarray] = []
        centre = est = None
        if lv is not None:
            t_w_t = lv.t_w_y @ self.t_y_t
            tags_w = [t_w_t.point(c) for c in tag_centres(self.cfg.apriltag).values()]
            centre, est = t_w_t.pos, lv.t_w_y.pos
        pred = self.predicted()
        ee = self.ee_pose()
        cc = self.task.cfg.capture
        attached = self.capture.is_attached(0)
        # Display only: once the MEP is held there is no target any more (`self.goal` stays at
        # its last approach value), so the target marker follows the EE instead of lagging it
        goal = ee if attached else self.goal
        if attached:
            col = cc.color_captured
        elif self.state in (State.SLOW_APPROACH, State.CAPTURE_ATTEMPT):
            col = cc.color_approach
        else:
            col = cc.color_idle
        if col != self._cylinder_color:  # capture cylinder: blue -> yellow (closing in) -> green (captured)
            self._cylinder_color = col
            self.capture.set_color(0, col)
        ## Orientation triads (X red, Y green, Z blue): estimated current Cylinder_01
        ## (bright, long), predicted (dim, long), GT (thin), EE and EE target (short)
        est_now = self.estimate_now()
        gt = self.gt_cylinder()
        axes = [(est_now, 0.35, 4, 1.0), (pred, 0.35, 4, 0.45), (gt, 0.25, 1, 1.0), (ee, 0.2, 2, 1.0), (goal, 0.2, 1, 0.6)]
        vectors = []
        v = self.predictor.velocity()
        if est_now is not None and v is not None:
            vectors.append((est_now.pos, est_now.pos + v * self.vis.VELOCITY_ARROW_SEC, "velocity", 3))
        if est_now is not None and pred is not None:
            vectors.append((est_now.pos, pred.pos, "pred", 2))  # future position prediction
        self.vis.set_scene(tags_w, centre, est, gt.pos, pred.pos if pred else None, ee.pos, goal.pos if goal else None,
                           axes=axes, vectors=vectors)

    def save_overlay(self, vf: VisionFrame):
        """2D overlay on the `cam_wrist` image (OpenCV)."""
        import cv2

        img = cv2.cvtColor(vf.image, cv2.COLOR_RGB2BGR)
        h, w = img.shape[:2]
        centers = []
        for tid in self.cfg.apriltag.ids:
            d = vf.detections.get(tid)
            if d is None:
                continue
            pts = d.corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [pts], True, (0, 255, 0), 2)
            cv2.circle(img, tuple(int(x) for x in d.corners[0]), 4, (0, 0, 255), -1)  # corner 0
            centers.append(d.center)
            cv2.putText(img, f"Tag {tid}", tuple(int(x) for x in d.corners[0] + np.array([0, -8])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if len(centers) >= 2:
            cv2.polylines(img, [np.array(centers, dtype=np.int32).reshape(-1, 1, 2)], True, (0, 255, 255), 2)
        cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 40, 1)
        cam_inv = vf.cam_w.inv()

        def proj(p_w):
            pc = cam_inv.point(p_w)
            if pc[2] <= 0.01:
                return None
            uv = project_points(pc, self.k)[0]
            return int(round(uv[0])), int(round(uv[1]))

        gt = proj(self.gt_cylinder().pos)
        if gt:
            cv2.circle(img, gt, 9, (0, 255, 0), 2)
        if vf.t_w_y is not None:
            e = proj(vf.t_w_y.pos)
            if e:
                cv2.drawMarker(img, e, (0, 140, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
        pred = self.predicted()
        if pred is not None:
            p = proj(pred.pos)
            if p:
                cv2.drawMarker(img, p, (255, 0, 255), cv2.MARKER_DIAMOND, 18, 2)

        def draw_axes(f: Optional[Frame], length: float, thickness: int, bright: float):
            """Local XYZ axes of `f` (X red, Y green, Z blue in BGR)."""
            if f is None:
                return
            o = proj(f.pos)
            for i, bgr in enumerate(((0, 0, 255), (0, 255, 0), (255, 0, 0))):
                e = proj(f.pos + length * f.rot[:, i])
                if o and e:
                    cv2.line(img, o, e, tuple(int(bright * c) for c in bgr), thickness, cv2.LINE_AA)

        # current estimate (bright, thick) vs prediction (dim, thin): the rotation over
        # the horizon shows as the offset between the two triads
        draw_axes(self.estimate_now(), 0.12, 3, 1.0)
        draw_axes(pred, 0.12, 1, 0.6)
        err = ""
        if vf.valid:
            pe, ae = pose_errors(self.gt_cylinder(), vf.t_w_y)
            err = f" | err {pe*1000:.2f} mm {ae:.3f} deg | rms {vf.reproj_rms:.2f} px"
        lines = [
            f"t={vf.t:6.2f}s  {self.state.value}  tags {len(vf.detections)}/4{err}",
            "green O: Cylinder_01 GT   orange X: estimate   magenta <>: prediction (+%.1fs)" % self.cfg.prediction.horizon_sec,
            "axes XYZ=RGB: thick = estimate now, thin/dim = prediction   [%s]" % self.cfg.mep.motion_mode,
        ]
        for i, text in enumerate(lines):
            cv2.putText(img, text, (12, 26 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(img, text, (12, 26 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        path = self.overlay_dir / f"{self.label}_{vf.t:07.2f}s.png"
        cv2.imwrite(str(path), img)
        self._overlays.append(path)

    def publish_ros(self):
        """Telemetry on the ROS topics (`ros_interface.py`). Ground truth goes to `gt/` only."""
        ee = self.ee_pose()
        est, pred = self.estimate_now(), self.predicted()
        v, w = self.predictor.twist()
        gt = self.gt_cylinder()
        gt_v, gt_w = self.gt_mep_velocity_at(gt.pos)
        self.ros.publish_poses(self.sim_time, est, pred, ee, self.goal, self.cam_pose(), v=v, w=w, gt=gt, gt_v=gt_v, gt_w=gt_w)
        m = self.est_capture_metrics()
        lv = self.last_vision
        ok, why = self.tracking_ok()
        self.ros.publish_state(self.sim_time, self.state.value, self.capture.is_attached(0), {
            "scenario": self.scenario, "motion_mode": self.cfg.mep.motion_mode,
            "tags": len(lv.detections) if lv else 0, "pose_valid": bool(lv.valid) if lv else False,
            "tracking_ok": ok, "tracking_reason": why, "standoff_m": self.standoff,
            "est_age_s": self.estimate_age(), "capture_enabled": self.ros.capture_enabled,
            "start_received": self.ros.start_received, "failure": self.results.failure,
            **{f"est_{k}": val for k, val in m.items()},
            **({"docked": bool(self.task.docking.is_docked)} if self.cfg.docking.enabled else {}),
            **({"orbit_error_m": self.orbit_error()[0]} if self.orbit is not None else {}),
        })
        if self.cfg.docking.enabled:
            self.ros.publish_docking(self.sim_time, bool(self.task.docking.is_docked), self.dock_world(),
                                     self.orbit_error()[0] if self.orbit is not None else None)

    def status(self):
        lv = self.last_vision
        m = self.capture_metrics(self.gt_cylinder(), self.gt_mep_velocity_at(self.ee_pose().pos)[0])
        print(f"[STATUS] t={self.sim_time:6.1f}s {self.state.value:<16} tags {len(lv.detections) if lv else 0}/4 "
              f"standoff {self.standoff:.3f} m | GT: dist {m['distance']:.3f} m gap {m['gap']*1000:7.1f} mm lat {m['lateral']*1000:6.1f} mm "
              f"ang {m['angle']:.2f} deg rel.v {m['rel_vel']*1000:.1f} mm/s | wall {time.time() - self.wall_start:.0f} s", flush=True)

    ###################
    ### Main loop ###
    ###################

    def open_wrist_view(self):
        """GUI: a live `cam_wrist` viewport window (the view the vision system uses)."""
        if self.headless:
            return
        try:
            from omni.kit.viewport.utility import create_viewport_window
            from pxr import Sdf

            self._wrist_window = create_viewport_window(
                "cam_wrist", width=640, height=360, position_x=40, position_y=60,
                camera_path=Sdf.Path(f"{self.geo.link_path}/{self.cfg.camera.name}"),
            )
            print("[VIS] opened the 'cam_wrist' viewport window", flush=True)
            # Docking phase: the probe's own view, under the wrist window
            pc = self.cfg.probe_camera
            if self.cfg.docking.enabled and pc.enabled:
                self._probe_window = create_viewport_window(
                    pc.name, width=640, height=480, position_x=40, position_y=460,
                    camera_path=Sdf.Path(f"{self.geo.mep_path}/{pc.name}"),
                )
                print(f"[VIS] opened the '{pc.name}' viewport window", flush=True)
        except Exception as e:  # never let a GUI convenience stop the run
            print(f"[VIS] could not open the camera window(s): {e} -- pick the camera in the viewport camera menu", flush=True)

    def run(self) -> Results:
        sim, scene = self.sim, self.scene
        self.open_wrist_view()
        vision_period = 1.0 / self.cfg.vision.rate_hz
        n = 0
        self.start()
        with torch.no_grad():
            while self.sim_app.is_running() and not self.done:
                if not sim.is_playing():
                    sim.render()
                    continue
                self.step()
                scene.write_data_to_sim()
                sim.step(render=False)
                n += 1
                self.sim_time += self.dt
                rendered = n % self.render_interval == 0
                # The docking phase does not use the wrist camera / AprilTags at all
                vision_due = (rendered and self.sim_time >= self._next_vision_t - 1e-9
                              and self.state not in DOCKING_STATES and not self._observe_only)
                if rendered:
                    if vision_due:
                        self.vis.clear()  # keep debug draw out of the image used for detection
                    # headless + camera.render_on_vision_only: nothing looks at the other frames
                    if vision_due or not (self.headless and self.cfg.camera.render_on_vision_only):
                        sim.render()
                scene.update(dt=self.dt)
                if vision_due:
                    self._next_vision_t = self.sim_time + vision_period - 1e-9
                    self.process_image()
                    self.update_visuals()
                    self.vis.show()
                    if self.ros is not None and self.last_vision is not None and self.last_vision.image is not None:
                        self.ros.publish_image(self.sim_time, self.last_vision.image)
                if self.ros is not None and self.sim_time >= self._next_ros_t:
                    self._next_ros_t = self.sim_time + 1.0 / self.cfg.ros.publish_rate_hz - 1e-9
                    self.publish_ros()
                if self.sim_time >= self._next_log_t:
                    self._next_log_t = self.sim_time + 1.0 / self.cfg.logging.rate_hz - 1e-9
                    self.log_row()
                    if self.orbit is not None:
                        self.log_orbit_row()
                if self.sim_time >= self._next_status_t and self.state not in DOCKING_STATES:
                    self._next_status_t = self.sim_time + 2.0
                    self.status()
        self.vis.clear()
        if self._csv_file is not None:
            self._csv_file.close()
        if self._dock_csv_file is not None:
            self._dock_csv_file.close()
        if self._orbit_csv_file is not None:
            self._orbit_csv_file.close()
        if self.ros is not None:
            self.publish_ros()  # final state (latched topics keep it for late subscribers)
            self.ros.close()
        return self.finish()

    ###############
    ### Results ###
    ###############

    def finish(self) -> Results:
        c = self.cfg
        r = self.results
        r.final_state = self.state.value
        r.metrics["sim_time_s"] = self.sim_time
        r.metrics["wall_time_s"] = time.time() - self.wall_start
        r.metrics["max_arm_contact_force_n"] = self._max_contact
        rows = self.rows
        valid = [x for x in rows if not math.isnan(x["position_error_mm"])]
        r.metrics["csv"] = str(self.csv_path)
        r.metrics["csv_rows"] = len(rows)
        r.metrics["overlay_images"] = len(self._overlays)
        r.metrics["overlay_dir"] = str(self.overlay_dir)
        r.metrics["debug_draw_primitives"] = self.vis.num_drawn
        r.metrics["pnp_ambiguous_images"] = self._ambiguous
        if valid:
            pe = np.array([x["position_error_mm"] for x in valid])
            ae = np.array([x["angle_error_deg"] for x in valid])
            r.metrics["tracking_error"] = {
                "position_error_mm_mean": float(pe.mean()), "position_error_mm_max": float(pe.max()),
                "angle_error_deg_mean": float(ae.mean()), "angle_error_deg_max": float(ae.max()),
                "samples": len(valid),
            }
            raw = [x for x in valid if not math.isnan(x["raw_position_error_mm"])]
            if raw:
                r.metrics["tracking_error"].update({
                    "single_image_position_error_mm_max": float(max(x["raw_position_error_mm"] for x in raw)),
                    "single_image_angle_error_deg_max": float(max(x["raw_angle_error_deg"] for x in raw)),
                })

        if c.logging.csv_enabled:
            with open(self.csv_path) as f:
                header = next(csv.reader(f))
            missing = [col for col in CSV_COLUMNS[:36] if col not in header]
            csv_ok = len(rows) > 0 and not missing
        else:  # CSV switched off: the "CSV generated" checks are skipped
            missing, csv_ok = [], None
            r.metrics["csv"] = "disabled (logging.csv_enabled: false)"

        if self._observe_only:
            r.metrics["capture_phase"] = "skipped (--orbit-only: scene check only)"
        elif c.docking.enabled and c.docking.skip_capture:
            # The capture phase never ran, so its checks would only report their own
            # absence; the capture is covered by the other scenarios.
            r.metrics["capture_phase"] = "skipped (docking.skip_capture)"
        elif self.scenario == "static":
            self._finish_static(csv_ok, missing)
        else:
            self._finish_dynamic(csv_ok, missing)
        if c.docking.enabled:
            self._finish_docking()
        if self.orbit is not None:
            self._finish_orbit()
        self._write_video()
        return r

    def _finish_static(self, csv_ok, missing):
        c = self.cfg
        r = self.results
        stats = self._static_stats
        r.metrics["static"] = stats
        det_ok = bool(stats) and all(s["samples"] >= c.test.static_samples for s in stats)
        r.check("[TEST1] AprilTag detection", det_ok and self.state == State.SUCCESS,
                f"4/4 tags with a valid PnP pose in {sum(s['samples'] for s in stats)} images at standoffs {[s['standoff_m'] for s in stats]}")
        if stats:
            worst_p = max(s["position_error_mm_max"] for s in stats)
            worst_a = max(s["angle_error_deg_max"] for s in stats)
        else:
            worst_p = worst_a = math.nan
        rms = max((s["reproj_rms_px_mean"] for s in stats), default=math.nan)
        r.check("[TEST1] PnP pose estimation", bool(stats) and rms <= c.apriltag.max_reprojection_rms_px, f"mean reprojection RMS <= {rms:.3f} px (16 points)")
        r.check("[TEST1] Cylinder_01 center estimation", bool(stats) and worst_p < c.test.static_position_threshold_mm and worst_a < c.test.static_angle_threshold_deg,
                f"worst position error {worst_p:.3f} mm (< {c.test.static_position_threshold_mm}), worst angle error {worst_a:.4f} deg (< {c.test.static_angle_threshold_deg})",
                position_error_mm=worst_p, angle_error_deg=worst_a)
        r.check("[TEST1] Debug visualization", self.vis.available and self.vis.num_drawn > 0 and len(self._overlays) > 0,
                f"debug draw primitives {self.vis.num_drawn}, overlay images {len(self._overlays)} in {self.overlay_dir}")
        if csv_ok is not None:
            r.check("[TEST1] CSV generated", csv_ok, f"{len(self.rows)} rows, {'all required columns' if not missing else 'missing ' + str(missing)} -> {self.csv_path}")
        passed = all(v["pass"] for k, v in r.checks.items() if k.startswith("[TEST1]")) and self.state == State.SUCCESS
        r.metrics["test1_pass"] = passed

    def _finish_dynamic(self, csv_ok, missing):
        c = self.cfg
        r = self.results
        ci = self._capture_info
        captured = bool(ci)
        cap_t = ci.get("time_s", math.nan)
        rel_gt = ci["gt"]["rel_vel"] if captured else math.nan
        rel_est = ci["vision"]["rel_vel"] if captured else math.nan
        r.metrics["capture"] = ci
        pre = [x for x in self.rows if x["capture_state"] in (State.APPROACHING.value, State.SLOW_APPROACH.value, State.CAPTURE_ATTEMPT.value)]
        tags_ok = bool(pre) and all(x["num_detected_tags"] == 4 for x in pre)
        r.check("[TEST2] AprilTag detection during approach", tags_ok, f"{sum(1 for x in pre if x['num_detected_tags'] == 4)}/{len(pre)} logged frames with 4/4 tags")
        pred_err = self._prediction_error()
        r.metrics["prediction_error"] = pred_err
        r.check("[TEST2] Future position prediction", pred_err.get("samples", 0) > 0,
                f"{c.prediction.horizon_sec:.1f} s prediction vs GT at t+h: mean {pred_err.get('mean_mm', math.nan):.2f} mm, max {pred_err.get('max_mm', math.nan):.2f} mm ({pred_err.get('samples', 0)} samples)")
        r.check("[TEST2] FixedJoint within timeout", captured and cap_t <= c.test.dynamic_timeout_sec,
                f"captured at t={cap_t:.2f} s (limit {c.test.dynamic_timeout_sec:.0f} s)", capture_time_s=cap_t)
        r.check("[TEST2] Relative velocity at capture", captured and rel_gt < c.capture.max_relative_velocity_mps,
                f"GT {rel_gt:.4f} m/s, vision estimate {rel_est:.4f} m/s (< {c.capture.max_relative_velocity_mps})", relative_velocity_mps=rel_gt)
        if captured:
            g = ci["gt"]
            r.check("[TEST2] Capture geometry (GT)", g["distance"] <= c.capture.max_distance_m and g["angle"] <= c.capture.max_angle_deg and g["lateral"] <= c.capture.max_lateral_m,
                    f"distance {g['distance']:.4f} m (<= {c.capture.max_distance_m}), angle {g['angle']:.3f} deg (<= {c.capture.max_angle_deg}), lateral {g['lateral']*1000:.2f} mm (<= {c.capture.max_lateral_m*1000:.0f})")
        if self.six_dof:
            self._finish_six_dof(captured)
        h = self._hold
        held = "duration_s" in h
        r.metrics["holding"] = {k: v for k, v in h.items() if k != "ee_start"}
        r.check("[TEST3] Holding duration", held and h["duration_s"] >= c.test.holding_duration_sec - 1e-6,
                f"{h.get('duration_s', 0.0):.2f} s (>= {c.test.holding_duration_sec})", holding_duration_s=h.get("duration_s", 0.0))
        r.check("[TEST3] FixedJoint validity", held and h.get("joint_valid", False), "joint prim present during the whole holding" if held else "not reached")
        stable = held and h["max_drift_mm"] < c.test.holding_max_relative_drift_mm and h["max_drift_deg"] < c.test.holding_max_relative_drift_deg and self.results.failure is None
        r.check("[TEST3] Physics stability (MEP-EE relative pose)", stable,
                f"max drift {h.get('max_drift_mm', math.nan):.3f} mm / {h.get('max_drift_deg', math.nan):.4f} deg, max MEP rate {h.get('max_mep_rate_deg_s', math.nan):.4f} deg/s, no NaN/explosion" if held else "not reached")
        if held:
            v0 = ci.get("mep_speed_mps", math.nan)
            v2 = h.get("max_mep_speed_second_half", math.nan)
            r.check("[TEST3] Robot EE stability (motion bounded, not diverging)", h["max_ee_motion_mm"] < 50.0 and v2 < v0,
                    f"max EE excursion {h['max_ee_motion_mm']:.1f} mm (< 50), MEP speed {v0*1000:.2f} mm/s at capture -> max {v2*1000:.2f} mm/s in the 2nd half of holding, "
                    f"{h['final_mep_speed']*1000:.2f} mm/s at the end (residual arm+payload oscillation), MEP moved {h['mep_displacement_mm']:.1f} mm after capture")
        rt = self._retreat
        if c.test.retreat_distance_m > 0.0:
            ok = rt is not None and "mep_moved_m" in rt and abs(rt["mep_moved_m"] - rt["ee_moved_m"]) < 0.005 and rt["max_drift_mm"] < c.test.holding_max_relative_drift_mm and rt["ee_moved_m"] > 0.8 * c.test.retreat_distance_m
            r.metrics["retreat"] = {k: v for k, v in (rt or {}).items() if k not in ("start", "mep_start")}
            r.check("[TEST3] Retreat with the MEP attached", ok,
                    f"EE {rt.get('ee_moved_m', math.nan)*1000:.1f} mm, MEP {rt.get('mep_moved_m', math.nan)*1000:.1f} mm, max drift {rt.get('max_drift_mm', math.nan):.3f} mm" if rt else "not reached")
        if csv_ok is not None:
            r.check("[DYN] CSV generated", csv_ok, f"{len(self.rows)} rows -> {self.csv_path}")
        r.metrics["test2_pass"] = all(v["pass"] for k, v in r.checks.items() if k.startswith("[TEST2]"))
        r.metrics["test3_pass"] = all(v["pass"] for k, v in r.checks.items() if k.startswith("[TEST3]")) and self.state == State.SUCCESS

    def _prediction_error(self) -> dict:
        """Compare each logged prediction (made at t for t+h) with the GT at t+h
        (position: linear interpolation, orientation: slerp between the logged GT rows)."""
        h = self.cfg.prediction.horizon_sec
        rows = [x for x in self.rows if not math.isnan(x["pred_x"]) and x["capture_success"] == 0]
        t = np.array([x["timestamp"] for x in self.rows])
        gt = np.array([[x["gt_x"], x["gt_y"], x["gt_z"]] for x in self.rows])
        errs, ang_errs, hold_errs = [], [], []
        for x in rows:
            tq = x["timestamp"] + h
            if tq > t[-1]:
                continue
            g = np.array([np.interp(tq, t, gt[:, i]) for i in range(3)])
            errs.append(1000.0 * float(np.linalg.norm(np.array([x["pred_x"], x["pred_y"], x["pred_z"]]) - g)))
            if math.isnan(x["pred_qw"]) or math.isnan(x["est_qw"]):
                continue
            i = int(np.clip(np.searchsorted(t, tq), 1, len(t) - 1))
            a, b = self.rows[i - 1], self.rows[i]
            s = 0.0 if b["timestamp"] <= a["timestamp"] else (tq - a["timestamp"]) / (b["timestamp"] - a["timestamp"])
            g_rot = interp_frame(*(Frame.from_pos_quat(np.zeros(3), [r[f"gt_q{k}"] for k in "wxyz"]) for r in (a, b)), float(np.clip(s, 0.0, 1.0))).rot
            pred_rot = Frame.from_pos_quat(np.zeros(3), [x[f"pred_q{k}"] for k in "wxyz"]).rot
            est_rot = Frame.from_pos_quat(np.zeros(3), [x[f"est_q{k}"] for k in "wxyz"]).rot
            ang_errs.append(math.degrees(rotation_angle(pred_rot, g_rot)))
            # baseline: holding the current estimate (no rotation model) for the horizon
            hold_errs.append(math.degrees(rotation_angle(est_rot, g_rot)))
        if not errs:
            return {"samples": 0}
        out = {"samples": len(errs), "mean_mm": float(np.mean(errs)), "max_mm": float(np.max(errs))}
        if ang_errs:
            out.update({"orientation_samples": len(ang_errs), "orientation_mean_deg": float(np.mean(ang_errs)), "orientation_max_deg": float(np.max(ang_errs)),
                        "hold_orientation_mean_deg": float(np.mean(hold_errs)), "hold_orientation_max_deg": float(np.max(hold_errs))})
        return out

    def _finish_six_dof(self, captured: bool):
        """Additional six_dof checks. The '(report)' checks only require the data to exist
        (no accuracy threshold has been agreed yet); the values are in the detail/metrics."""
        c = self.cfg
        r = self.results
        fr = self._w_frame_result
        r.metrics["angular_velocity_frame"] = fr
        r.check("[6DOF] Angular velocity is a world-frame vector", bool(fr and fr["pass"]),
                (f"direction error {fr['direction_error_deg']:.2f} deg (< 5); from poses (world) error {fr['err_world']:.2e}, (body) error {fr['err_body']:.2e} rad/s"
                 f"{'' if fr['world_and_body_distinguishable'] else ', world/body not distinguishable for this start orientation'}") if fr else "not measured")
        r.check("[6DOF] Rotation rate vs commanded (report)", bool(fr),
                (f"rate from poses / commanded = {fr['rate_ratio_pose_over_cmd']:.3f}, / root_com_ang_vel_w = {fr['rate_ratio_pose_over_root_com']:.3f} "
                 "(1.0 = the body turns at the reported rate)") if fr else "not measured")
        rows = [x for x in self.rows if x["capture_success"] == 0 and not math.isnan(x["mep_angular_velocity_x"])]
        if rows:
            e = np.array([[x[f"mep_angular_velocity_{k}"] - x[f"gt_mep_angular_velocity_{k}"] for k in "xyz"] for x in rows])
            n = np.linalg.norm(e, axis=1)
            w_gt = np.array([[x[f"gt_mep_angular_velocity_{k}"] for k in "xyz"] for x in rows])
            stats = {"samples": len(rows), "error_mean_rad_s": float(n.mean()), "error_max_rad_s": float(n.max()),
                     "gt_rate_mean_rad_s": float(np.linalg.norm(w_gt, axis=1).mean())}
        else:
            stats = {"samples": 0}
        r.metrics["angular_velocity_error"] = stats
        r.check("[6DOF] Angular velocity estimation (report)", stats["samples"] > 0,
                f"|w_est - w_gt| mean {stats.get('error_mean_rad_s', math.nan):.5f}, max {stats.get('error_max_rad_s', math.nan):.5f} rad/s "
                f"(|w_gt| mean {stats.get('gt_rate_mean_rad_s', math.nan):.5f} rad/s, {stats['samples']} samples)")
        pe = r.metrics.get("prediction_error", {})
        r.check("[6DOF] Future orientation prediction (report)", pe.get("orientation_samples", 0) > 0,
                f"{c.prediction.horizon_sec:.1f} s prediction vs GT at t+h: mean {pe.get('orientation_mean_deg', math.nan):.4f} deg, max {pe.get('orientation_max_deg', math.nan):.4f} deg "
                f"(holding the current estimate instead: mean {pe.get('hold_orientation_mean_deg', math.nan):.4f} deg)")
        ci = self._capture_info
        rel = ci["gt"]["rel_ang_vel"] if captured else math.nan
        r.check("[6DOF] Relative angular velocity at capture", captured and rel <= c.capture.max_relative_angular_velocity_rad_s,
                f"GT {rel:.5f} rad/s, vision estimate {ci['vision']['rel_ang_vel'] if captured else math.nan:.5f} rad/s (<= {c.capture.max_relative_angular_velocity_rad_s})",
                relative_angular_velocity_rad_s=rel)

    def _finish_docking(self):
        """Docking-phase results: what actually happened, plus the depth cross-check."""
        d = self.cfg.docking
        r = self.results
        dk = self._dock
        docked = bool(self.task.docking.is_docked)
        r.metrics.setdefault("docking", {}).update({
            "docked": docked,
            "rows": len(self._dock_rows),
            "csv": str(getattr(self, "dock_csv_path", "")) if self.cfg.logging.csv_enabled else "disabled",
            "probe_rgb_frames": len(self._probe_frames),
            "probe_rgb_dir": str(self._probe_dir),
            "realign_events": int(dk.get("realigns", 0)),
            "depth_blocked_s": float(dk.get("depth_blocked_s", 0.0)),
            "min_wall_clearance_m": (None if not math.isfinite(float(dk.get("min_clearance", math.inf))) else float(dk["min_clearance"])),
            "max_lateral_while_inserting_m": float(dk.get("max_lateral_inserting", 0.0)),
            "at_dock": dk.get("at_dock"),
        })
        if self._dock_csv is not None:
            r.check("[DOCK] CSV generated", len(self._dock_rows) > 0,
                    f"{len(self._dock_rows)} rows -> {getattr(self, 'dock_csv_path', '')}")
        # Depth vs geometry, from the moment the camera is on the docking axis. Before
        # that the probe is still being swung 5.5 m sideways and the ray simply misses
        # the nozzle, so those readings say nothing about the sensor.
        since = self._depth_calibrated_t if self._depth_calibrated_t is not None else math.inf
        rows = [x for x in self._dock_rows
                if x["timestamp"] >= since and math.isfinite(x["depth_distance_m"]) and math.isfinite(x["geometry_distance_m"])]
        if rows:
            diff = np.array([abs(x["depth_distance_m"] - x["geometry_distance_m"]) for x in rows])
            geo = np.array([x["geometry_distance_m"] for x in rows])
            dep = np.array([x["depth_distance_m"] for x in rows])
            rejected = int((diff > d.depth_agreement_m).sum())
            stats = {"samples": len(rows), "mean_abs_difference_m": float(diff.mean()), "max_abs_difference_m": float(diff.max()),
                     "rejected_frames": rejected, "rejected_fraction": rejected / len(rows),
                     "geometry_first_m": float(geo[0]), "geometry_last_m": float(geo[-1]),
                     "depth_first_m": float(dep[0]), "depth_last_m": float(dep[-1]),
                     "correlation": float(np.corrcoef(geo, dep)[0, 1]) if len(rows) > 2 and geo.std() > 1e-9 and dep.std() > 1e-9 else None}
            r.metrics["docking"]["depth_vs_geometry"] = stats
            # Judged on what the controller was allowed to use: frames outside the
            # agreement band are rejected by `depth_valid` and the probe holds instead
            # of advancing, so a few of them are handled, not ignored.
            r.check("[DOCK3] Depth agrees with the geometric docking distance",
                    float(diff.mean()) <= d.depth_agreement_m and rejected <= 0.05 * len(rows),
                    f"|depth - geometry| mean {diff.mean()*1000:.0f} mm, max {diff.max()*1000:.0f} mm (band {d.depth_agreement_m*1000:.0f} mm), "
                    f"{rejected}/{len(rows)} frames outside the band were rejected and held the approach, correlation {stats['correlation']:.4f}")
            r.check("[DOCK3] Depth decreases during the approach",
                    float(dep[-1]) < float(dep[0]) and float(geo[-1]) < float(geo[0]),
                    f"depth {dep[0]:.3f} -> {dep[-1]:.3f} m, geometry {geo[0]:.3f} -> {geo[-1]:.3f} m")
        else:
            r.check("[DOCK3] Depth agrees with the geometric docking distance", False,
                    "the probe never reached the docking axis, so the depth camera was never on the nozzle"
                    if self._depth_calibrated_t is None else f"no usable depth reading ({self._depth_reason})")
        r.check("[DOCK5] Docking FixedJoint created", docked,
                r.checks.get("[DOCK5] Docking FixedJoint created", {}).get("detail", "the probe never reached the docking conditions"))
        if not docked:
            r.failure = r.failure or f"docking did not complete (final state {self.state.value})"

    def _finish_orbit(self):
        """Orbit-scenario results: free flight over the whole run, and what the return phase reached."""
        c, oc, r, h = self.cfg, self.orbit_cfg, self.results, self._orbit
        ff = self._free_flight
        tol_v, tol_w = c.drift.velocity_tolerance_mps, c.drift.angular_tolerance_rad_s
        ff_ok = bool(ff) and all(x["max_dv_mps"] <= tol_v and x["max_w_rad_s"] <= tol_w for x in ff.values())
        r.check("[ORBIT0] Free flight held until captured / docked (no rotation, no velocity change)", ff_ok,
                "; ".join(f"{k}: max |v - v_cmd| {x['max_dv_mps']*1000:.3f} mm/s, max |w| {x['max_w_rad_s']:.6f} rad/s over {x['samples']} steps" for k, x in ff.items())
                + f" (tolerance {tol_v*1000:.1f} mm/s, {tol_w:.4f} rad/s)" if ff else "the free-flight monitor never ran")
        m = r.metrics.setdefault("orbit", {})
        m["free_flight"] = ff
        m["return_phase"] = "run" if self.orbit_return_active else ("skipped (--orbit-only)" if self._observe_only else "not requested (no docking)")
        if self._orbit_csv is not None:
            r.check("[ORBIT] CSV generated", len(self._orbit_rows) > 0, f"{len(self._orbit_rows)} rows -> {getattr(self, 'orbit_csv_path', '')}")
        if not self.orbit_return_active:
            return
        m.update({k: v for k, v in h.items() if k not in ("prev_ref",)})
        if "target" in m:
            m["target"] = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in m["target"].items()}
        # Everything the request lists must have been judged: anything not reached counts as a failure
        for name in ("[ORBIT1] Orbit target acquired", "[ORBIT2] Transfer with both FixedJoints kept, no jump",
                     "[ORBIT3] Client point on the orbit", "[ORBIT4] Held on the orbit"):
            if name not in r.checks:
                r.check(name, False, f"not reached (final state {self.state.value})")
        if self.state != State.SUCCESS:
            r.failure = r.failure or f"orbit return did not complete (final state {self.state.value})"

    def _write_video(self):
        import cv2

        if not self._overlays:
            return
        first = cv2.imread(str(self._overlays[0]))
        h, w = first.shape[:2]
        path = self.out_dir / f"{self.label}_overlay.mp4"
        vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 4, (w, h))
        if vw.isOpened():
            for p in self._overlays:
                vw.write(cv2.imread(str(p)))
            vw.release()
            self.results.metrics["overlay_video"] = str(path)

    def write_json(self, path: Path):
        r = self.results
        payload = {"scenario": r.scenario, "run": self.label, "final_state": r.final_state, "failure": r.failure, "checks": r.checks, "metrics": r.metrics}
        path.write_text(json.dumps(payload, indent=2, default=_json_default))


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, float) and math.isnan(o):
        return None
    return str(o)


def default_out_dir() -> Path:
    return PROJECT_DIR.joinpath("logs", "vision_capture")
