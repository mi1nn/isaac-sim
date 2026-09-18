#!/usr/bin/env python3
"""Vision-guided magnet capture of the MEP, then peg-in-hole into the thruster.

Runs outside of Isaac Sim (system ROS 2), next to:
    srb agent ros --interface ros --env debris_capture_visual \
        --kit_args "--ext-folder /home/rokey/isaac-sim/apps --enable isaacsim.exp.base" \
        env.robot=canadarm3 env.sim.device=cuda

One run covers both phases:

  PHASE 1 (capture)  SEARCH -> APPROACH -> ALIGN -> DOCK -> SETTLE -> VERIFY -> HOLD
      The wrist camera finds the MEP's green capture marker and the arm flies its
      capture cylinder onto it. `CaptureManager` (srb/tasks/manipulation/
      debris_capture/capture.py) welds the two with a fixed joint as soon as the
      centres are within `CaptureCfg.distance_threshold`. There is no "close the
      gripper" topic: the job is purely to fly the capture point onto the marker
      squarely, and then stop pushing.

  PHASE 2 (peg-in-hole)  RING_SEARCH -> RING_ALIGN -> INSERT -> INSERTED
      The MEP's own camera, mounted beside the peg, finds the green ring around the
      satellite's thruster nozzle, and the arm flies the peg into it.

Why ALIGN exists
    The marker is a puck whose axis IS the MEP's long axis, so whatever tilt the arm
    has when the weld fires becomes a permanent tilt of the peg -- and the peg tip is
    10.2 m from the capture point, where one degree is 0.18 m against a hole that
    offers about 0.30 m of room. Docking straight at the marker centre (what
    APPROACH does) leaves that tilt to chance. ALIGN instead fits a plane to the
    near half of the marker's point cloud, which is its flat front face, and lines
    the arm up with that face's normal before DOCK drives in. `CaptureManager` logs
    the tilt it actually welded at, so the result is measurable rather than assumed.

Why the ring
    A nozzle is a dark hole on a dark bus -- nothing for a colour detector to hold
    on to. `VisualTaskCfg.thruster_ring_*` puts an emissive green annulus in front of
    the mouth, painted exactly like the capture marker, so ONE detector serves both
    phases. A ring beats a blob: fitting a circle to it yields the hole's centre AND
    its axis, and the fit still works from a partial arc, which matters because the
    far side of the ring leaves the frame as the peg closes in.

  * No gripper, and therefore no `srb/env0/end_effector` frame. The ROS interface
    broadcasts one frame per scene asset and one per articulation link, but skips
    FrameTransformer sensors such as `tf_end_effector`, and `scene.end_effector` is
    None when the robot carries no tool. Everything here is therefore anchored on
    `srb/env0/robot/canadarm3_large_7`, the arm's last link, whose pose comes
    straight from PhysX. Link 7 reaches along its own -Z, which is where the flange
    (0.44 m), the capture cylinder centre (0.64 m) and the wrist camera (0.9 m) all
    sit. The sign is a parameter (`approach_axis_sign`) rather than a buried
    constant, because getting it wrong drives the arm backwards.

Frames (all published by the SRB ROS interface)
    srb/env0                            env frame (all control math happens here)
    srb/env0/robot                      Canadarm3 base (IK deltas are in this frame)
    srb/env0/robot/canadarm3_large_7    last link (-Z = the direction the arm reaches)
    srb/env0/debris                     the MEP body

NOTE: An earlier revision of this file claimed there was no MEP frame. There is:
`ros.py::_broadcast_transforms` walks `scene._rigid_objects`, and the MEP is
registered there under the name "debris", not "mep". Its pose is `data.root_pos_w`,
i.e. straight from PhysX, so it is exactly as trustworthy as the link frames. Phase 2
depends on it: the MEP camera and the peg tip are fixed offsets in that frame.

NOTE: The TF of "srb/env0/cam_wrist" (and of "cam_mep") is NOT used, because Isaac Lab
reads camera poses from USD, which does not follow a moving articulation. Both camera
poses are derived from a body frame plus the known mount instead.
"""

import enum
import math
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vision_servo import (  # noqa: E402
    angle_between,
    axis_angle,
    backproject,
    clamp_norm,
    detect_color_blob,
    fit_circle_3d,
    fit_plane_ransac,
    near_face_points,
    normalize,
)

QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class State(enum.Enum):
    ## Phase 1: capture
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    DOCK = "DOCK"
    SETTLE = "SETTLE"
    VERIFY = "VERIFY"
    HOLD = "HOLD"
    ## Phase 2: peg-in-hole
    RING_SEARCH = "RING_SEARCH"
    RING_ALIGN = "RING_ALIGN"
    INSERT = "INSERT"
    INSERTED = "INSERTED"
    DONE = "DONE"


## Math helpers ##


def rot_x(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_matrix(rpy_deg) -> np.ndarray:
    """USD's xformOp:rotateXYZ, i.e. rotate about X, then Y, then Z."""
    return rot_z(rpy_deg[2]) @ rot_y(rpy_deg[1]) @ rot_x(rpy_deg[0])


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


## Camera plumbing ##


class CameraStream:
    """RGB + depth + intrinsics for one camera, with new-frame detection."""

    def __init__(self, node: Node, ns: str, name: str):
        self.name = name
        self.rgb: Optional[Image] = None
        self.depth: Optional[Image] = None
        self.K: Optional[np.ndarray] = None
        self._last_stamp: Optional[Tuple[int, int]] = None
        node.create_subscription(
            Image, f"{ns}/{name}/image_rgb", self._cb_rgb, QOS
        )
        node.create_subscription(
            Image, f"{ns}/{name}/image_depth", self._cb_depth, QOS
        )
        node.create_subscription(
            CameraInfo, f"{ns}/{name}/camera_info", self._cb_info, QOS
        )

    def _cb_rgb(self, msg: Image):
        self.rgb = msg

    def _cb_depth(self, msg: Image):
        self.depth = msg

    def _cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def take_frame(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """The newest unprocessed (rgb, depth) pair, or None."""
        if self.rgb is None or self.depth is None or self.K is None:
            return None
        stamp = (self.rgb.header.stamp.sec, self.rgb.header.stamp.nanosec)
        if stamp == self._last_stamp:
            return None
        self._last_stamp = stamp
        rgb = image_to_numpy(self.rgb)
        depth = image_to_numpy(self.depth)
        if rgb is None or depth is None:
            return None
        return rgb, depth


def image_to_numpy(msg: Image) -> Optional[np.ndarray]:
    if msg.encoding in ("rgb8", "rgba8"):
        ch = 3 if msg.encoding == "rgb8" else 4
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
        return np.ascontiguousarray(img[:, :, :3])
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    return None


## Node ##


class MepSuctionCapture(Node):
    def __init__(self):
        super().__init__("mep_suction_capture")

        ## Parameters
        p = self.declare_parameter
        # Topics & frames
        p("env_ns", "/srb/env0")
        p("camera_name", "cam_wrist")
        p("mep_camera_name", "cam_mep")
        p("twist_topic", "/srb/env0/robot/robot/differential_inverse_kinematics")
        p("env_frame", "srb/env0")
        p("base_frame", "srb/env0/robot")
        # Link 7 of the arm -- NOT "srb/env0/end_effector", which does not exist here.
        # The ROS interface publishes a frame per scene asset and one per articulation
        # link, but `tf_end_effector` is a FrameTransformer sensor (skipped by
        # `_broadcast_transforms`) and `scene.end_effector` is None without a tool.
        # The link frame is the better reference anyway: it is the PhysX body pose, so
        # it cannot go stale the way a USD-derived pose can.
        p("link_frame", "srb/env0/robot/canadarm3_large_7")
        # The MEP rigid body. Named after the scene entity, hence "debris".
        p("debris_frame", "srb/env0/debris")

        # Kinematics, all measured from link 7 along its -Z, which is the direction the
        # arm reaches: the flange sits at 0.44 m (canadarm3.py `frame_flange`), the
        # wrist camera at 0.9 m (`frame_wrist_camera`, overridden in task_visual.py).
        p("camera_pos_link7", [0.0, 0.0, -0.9])
        p("camera_rpy_link7", [0.0, 0.0, 0.0])  # USD/OpenGL camera convention
        # Multiplies link 7's +Z to give the reaching direction; -1 because the arm
        # reaches along -Z. Flip this if the arm servos away from the MEP.
        p("approach_axis_sign", -1.0)

        # Capture geometry. CaptureCfg spawns a cylinder of length `CaptureCfg.length`
        # (0.4 m) starting at the flange face and reaching outwards, so its centre --
        # the point that must meet the marker -- is 0.44 + 0.5 * 0.4 from link 7.
        p("capture_offset_link7", 0.64)
        # Must match CaptureCfg.distance_threshold; the arm aims well inside it.
        p("capture_distance_threshold", 0.3)
        p("dock_margin", 0.6)  # aim this fraction of the threshold from the marker

        # MEP geometry, in the MEP body frame (metres, i.e. asset units already
        # multiplied by the 1.1 debris spawn scale). Both are fixed offsets of the
        # rigid body, so `debris_frame` turns them into live env-frame poses.
        #   camera:  TaskCfg.debris_camera_xform, pushed through the telescope Xform
        #            (a pure 0.3 scale + translation) and then the 1.1 body scale
        #   peg tip: the free end of the Ares1 rocket body, TaskCfg.debris_peg_relpath
        p("mep_camera_pos_body", [-0.98527, 1.08438, 0.10615])
        p("mep_camera_rpy_body", [0.0, -90.0, -100.935287])  # USD rotateXYZ
        p("peg_tip_body", [-1.81027, -2.43562, 0.10615])
        p("peg_axis_body", [0.0, -1.0, 0.0])  # tip direction, i.e. where the peg points

        # Thruster ring -> hole. `VisualTaskCfg.thruster_ring_center` sits 0.04 asset
        # units in front of the nozzle mouth and the satellite spawns at scale 2.0, so
        # the mouth is this far behind the plane of the ring.
        p("ring_to_mouth", 0.08)
        p("ring_radius_nominal", 0.51)  # (0.42 + 0.60) / 2, for sanity checking a fit
        p("ring_radius_tolerance", 0.25)

        # Detection (OpenCV HSV: H in [0, 180]). One range for both targets: the ring
        # is painted the same green as the marker, and they never share a camera.
        p("hsv_lower", [45, 120, 60])
        p("hsv_upper", [75, 255, 255])
        p("min_pixels", 12)
        # Separate depth gates, because the marker and the ring share a colour and at
        # long range both are a handful of pixels: without this the wrist camera can
        # lock onto the ring 25 m away instead of the marker. The arm reaches ~8.5 m,
        # so nothing green beyond `marker_max_depth` can be the capture marker.
        p("marker_max_depth", 12.0)
        p("ring_max_depth", 30.0)
        p("estimate_smoothing", 0.5)  # EMA weight of a new measurement
        # Depth only sees the front half of the marker cylinder, whose mean lies
        # pi*r/4 in front of the axis -> push the estimate back along the viewing ray.
        # The marker measures ~0.165 m across after TaskCfg.debris_marker_xform, i.e.
        # a radius of ~0.082 m, scaled by the debris spawn scale (1.1).
        # Only used as the long-range fallback; once the face plane fits, the centre
        # comes from it instead (see `_update_marker_perception`).
        p("marker_radius", 0.09)
        # Half the marker puck's length: the capture point is its centre, the plane
        # fit finds its front face, and this is the distance between the two.
        p("marker_half_length", 0.055)
        # A plane fit needs enough of the face to be resolved; below this the blob is
        # a handful of pixels and the normal is noise.
        p("plane_fit_min_pixels", 120)
        p("plane_fit_near_fraction", 0.5)
        p("plane_fit_inlier_tol", 0.004)

        # Control
        #
        # SPEED: the fastest the arm can be driven is `ik_*_gain * max_action`, because
        # `_publish_command` divides the wanted velocity by the gain and then clamps the
        # result. Raising only `max_lin_vel` therefore buys nothing once the action
        # saturates -- `max_action` has to come up with it. Nothing clips the action on
        # its way into the simulator (the env declares a [-1, 1] Box but neither the ROS
        # interface nor `_pre_physics_step` enforces it), and the arm's own
        # `velocity_limit_sim` of 5 rad/s is the real backstop, so a ceiling of
        # 3 * 0.17 = 0.51 m/s is safe to ask for.
        p("control_rate", 30.0)
        p("ik_lin_gain", 0.17)  # [m/s] of motion per unit of IK action
        p("ik_rot_gain", 0.17)  # [rad/s] of motion per unit of IK action
        # Where the differential IK rotates about, from link 7's origin along its -Z
        # [m]. Canadarm3's action group uses body_offset (0, 0, -0.45), and with no
        # tool nothing overrides it.
        p("ik_control_point_link7", 0.45)
        p("max_action", 3.0)
        # Proportional gains: high enough that the arm sits at its velocity cap for most
        # of the travel instead of decaying towards the goal over the last metre.
        p("kp_lin", 1.2)
        p("kp_rot", 1.6)
        p("max_lin_vel", 0.36)
        p("max_rot_vel", 0.36)
        # DOCK stays far slower than APPROACH on purpose: this is the leg that ends in
        # contact, and the MEP is heavy enough that a fast nudge sends it tumbling.
        p("dock_lin_vel", 0.12)
        p("standoff", 0.6)
        p("approach_pos_tol", 0.05)
        p("approach_ang_tol_deg", 8.0)
        p("lock_distance", 1.0)  # stop updating the estimate when closer than this
        p("dock_timeout", 240.0)  # [s] DOCK without arriving -> back to APPROACH
        p("settle_duration", 1.0)  # let CaptureManager see a steady distance

        # ALIGN: square the arm up with the marker's face before driving in.
        p("align_ang_tol_deg", 1.5)
        p("align_lat_tol", 0.03)  # [m] off the marker's axis
        p("align_max_lin_vel", 0.10)
        p("align_timeout", 90.0)  # [s] give up squaring and dock anyway
        p("align_hold_duration", 0.5)  # stay inside tolerance this long before DOCK

        # Capture check: back off and see whether the marker follows the arm
        p("verify_distance", 0.12)  # [m], 0 disables the check
        p("verify_vel", 0.10)
        p("verify_timeout", 60.0)
        p("verify_follow_ratio", 0.7)  # above this the MEP is considered captured

        p("lost_timeout", 2.0)
        p("detections_to_start", 3)

        # Search: the arm swings about the base axis while looking outwards, first
        # towards -sweep (clockwise seen from +Z, where the MEP usually starts), then
        # back. Sweeping both ways keeps joint 1 within +-2pi, the PhysX drive limit.
        p("search_sweep_deg", 180.0)
        p("search_first_dir", -1.0)
        p("search_lead_deg", 30.0)  # the target always leads the actual azimuth
        p("search_max_rot_vel", 0.45)
        p("search_pitch_steps_deg", [0.0, 30.0, -30.0])
        p("reacquire_duration", 10.0)  # [s] look at the last known marker pose first

        ## Phase 2: peg-in-hole
        #
        # Every phase-2 gain is far smaller than its phase-1 twin for one reason: the
        # peg tip is ~11 m from the IK control point, so the arm's rotation is a lever.
        # 0.01 rad/s at the control point is 0.11 m/s at the tip -- already brisk for
        # something being threaded into a 0.33 m hole.
        p("insert_kp_lin", 0.8)
        p("insert_kp_rot", 0.35)
        p("insert_max_lin_vel", 0.08)
        p("insert_max_rot_vel", 0.012)
        p("insert_vel", 0.03)  # [m/s] advance along the hole axis while inserting
        p("insert_standoff", 1.2)  # [m] hold the tip this far out while aligning
        p("insert_ang_tol_deg", 1.0)
        p("insert_lat_tol", 0.05)  # [m] tip off the hole axis
        p("insert_hold_duration", 0.5)
        p("insert_depth", 0.8)  # [m] past the nozzle mouth before declaring success
        # Contact: the nozzle bore converges, so the peg stops well before `insert_depth`
        # if anything is off. Treat "commanded but not moving" as arrival.
        p("insert_stall_window", 2.5)  # [s]
        p("insert_stall_travel", 0.01)  # [m] of tip advance within the window
        p("ring_search_settle", 3.0)  # [s] hold still and just look, before scanning
        p("ring_scan_deg", 10.0)  # conical scan half-angle when the ring is not seen
        p("ring_scan_period", 24.0)  # [s] per revolution of the scan
        p("ring_lost_timeout", 3.0)
        p("phase2_enabled", True)

        # Debug
        p("publish_debug_image", True)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._g = g

        ## Fixed transforms, expressed directly in the link-7 frame.
        R_gl_ros = np.diag([1.0, -1.0, -1.0])  # OpenGL camera -> ROS optical frame
        self._R_l7_cam = rpy_to_matrix(g("camera_rpy_link7")) @ R_gl_ros
        self._p_l7_cam = np.array(g("camera_pos_link7"))
        ## ... and the MEP camera, in the MEP body frame.
        self._R_mep_cam = rpy_to_matrix(g("mep_camera_rpy_body")) @ R_gl_ros
        self._p_mep_cam = np.array(g("mep_camera_pos_body"))
        self._p_mep_peg = np.array(g("peg_tip_body"))
        self._d_mep_peg = normalize(np.array(g("peg_axis_body")))
        self.get_logger().info(
            f"Camera in link 7: p={np.round(self._p_l7_cam, 3)} "
            f"optical axis={np.round(self._R_l7_cam[:, 2], 3)} | "
            f"capture point {g('capture_offset_link7')} m, "
            f"IK control point {g('ik_control_point_link7')} m along the reach axis"
        )
        self.get_logger().info(
            f"Camera in MEP body: p={np.round(self._p_mep_cam, 3)} "
            f"optical axis={np.round(self._R_mep_cam[:, 2], 3)} | "
            f"peg tip {np.round(self._p_mep_peg, 3)} "
            f"pointing {np.round(self._d_mep_peg, 3)}"
        )

        ## ROS interfaces
        ns = g("env_ns").rstrip("/")
        self._cam_wrist = CameraStream(self, ns, g("camera_name"))
        self._cam_mep = CameraStream(self, ns, g("mep_camera_name"))
        self._pub_twist = self.create_publisher(Twist, g("twist_topic"), QOS)
        self._pub_state = self.create_publisher(String, "~/state", QOS)
        self._pub_marker = self.create_publisher(PointStamped, "~/marker", QOS)
        self._pub_hole = self.create_publisher(PointStamped, "~/hole", QOS)
        self._pub_debug = self.create_publisher(Image, "~/debug_image", QOS)
        self._pub_debug_mep = self.create_publisher(Image, "~/debug_image_mep", QOS)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        ## State
        self._state = State.SEARCH
        self._state_since = self.get_clock().now()
        # Phase 1 perception
        self._marker_pos: Optional[np.ndarray] = None
        self._marker_axis: Optional[np.ndarray] = None  # face normal, points at the arm
        self._last_detection_time: Optional[Time] = None
        self._detection_streak = 0
        self._locked = False
        self._dock_dir: Optional[np.ndarray] = None
        self._verify_start: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._search_origin: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._search_first_dir = math.copysign(1.0, g("search_first_dir"))
        self._search_dir = self._search_first_dir
        self._search_pitch_idx = 0
        self._in_tolerance_since: Optional[Time] = None
        # Phase 2 perception
        self._hole_pos: Optional[np.ndarray] = None  # nozzle mouth centre, env frame
        self._hole_axis: Optional[np.ndarray] = None  # points out of the hole, at the peg
        self._last_ring_time: Optional[Time] = None
        self._ring_streak = 0
        self._stall_ref: Optional[Tuple[Time, np.ndarray]] = None

        self.create_timer(1.0 / g("control_rate"), self._step)
        self.get_logger().info(
            "MEP suction capture node started (state: SEARCH). Capture is automatic: "
            f"CaptureManager welds the MEP once the capture point is within "
            f"{g('capture_distance_threshold')} m of the marker."
            + (
                " Peg-in-hole follows automatically once captured."
                if g("phase2_enabled")
                else " Phase 2 (peg-in-hole) is disabled."
            )
        )

    ## Main loop ##

    def _step(self):
        link = self._lookup(self._g("link_frame"))
        base = self._lookup(self._g("base_frame"))
        if link is None or base is None:
            self._warn_missing_frames(link is None, base is None)
            return
        p_f, R_f = link
        _, R_b = base

        # The reaching direction of the flange, and the capture cylinder's centre.
        approach = self._g("approach_axis_sign") * R_f[:, 2]
        capture_point = p_f + approach * self._g("capture_offset_link7")

        self._update_marker_perception(p_f, R_f)
        self._update_ring_perception()

        if self._state in (
            State.RING_SEARCH,
            State.RING_ALIGN,
            State.INSERT,
            State.INSERTED,
        ):
            v, w, point = self._step_phase2(p_f, R_f, approach, R_b)
        else:
            v, w = self._step_phase1(p_f, R_f, approach, capture_point, R_b)
            point = capture_point

        self._publish_command(v, w, point, R_b, p_f, approach)
        self._pub_state.publish(String(data=self._state.value))

    ## Phase 1: capture ##

    def _step_phase1(self, p_f, R_f, approach, capture_point, R_b):
        now = self.get_clock().now()
        detected_recently = self._last_detection_time is not None and (
            now - self._last_detection_time
        ) < Duration(seconds=self._g("lost_timeout"))
        v = np.zeros(3)
        w = np.zeros(3)

        if self._state == State.SEARCH:
            if self._marker_pos is not None and self._detection_streak >= self._g(
                "detections_to_start"
            ):
                self._set_state(State.APPROACH)
            else:
                v, w = self._search_motion(capture_point, approach, R_b, now)

        elif self._state == State.APPROACH:
            if not detected_recently:
                self._set_state(State.SEARCH)
            else:
                d = normalize(self._marker_pos - capture_point)
                goal = self._marker_pos - d * self._g("standoff")
                v, w = self._servo(
                    capture_point, R_f, goal, d, self._g("max_lin_vel")
                )
                pos_err = np.linalg.norm(goal - capture_point)
                ang_err = math.degrees(angle_between(approach, d))
                if pos_err < self._g("approach_pos_tol") and ang_err < self._g(
                    "approach_ang_tol_deg"
                ):
                    self._dock_dir = d
                    self._set_state(State.ALIGN)

        elif self._state == State.ALIGN:
            if not detected_recently:
                self._set_state(State.SEARCH)
            else:
                v, w = self._align_motion(capture_point, R_f, approach, now)

        elif self._state == State.DOCK:
            if not self._locked and np.linalg.norm(
                self._marker_pos - self._camera_pos(p_f, R_f)
            ) < self._g("lock_distance"):
                self._locked = True
                self.get_logger().info(
                    f"Marker estimate locked at {np.round(self._marker_pos, 3)}"
                )
            d = self._dock_dir
            # Stop short of the marker centre: the capture only needs the two points
            # inside the threshold, and driving all the way in would push the MEP away
            # before CaptureManager ever sees a small enough distance.
            aim = self._g("dock_margin") * self._g("capture_distance_threshold")
            goal = self._marker_pos - d * aim
            v, w = self._servo(capture_point, R_f, goal, d, self._g("dock_lin_vel"))
            gap = float(np.linalg.norm(self._marker_pos - capture_point))
            if gap < self._g("capture_distance_threshold"):
                self.get_logger().info(
                    f"Capture point within {gap:.3f} m of the marker "
                    f"(threshold {self._g('capture_distance_threshold')}) -> settling"
                )
                self._set_state(State.SETTLE)
            elif (now - self._state_since) > Duration(seconds=self._g("dock_timeout")):
                self.get_logger().warn("DOCK timed out -> re-approaching")
                self._set_state(State.APPROACH)

        elif self._state == State.SETTLE:
            # Hold still so CaptureManager sees a steady, small distance.
            if (now - self._state_since) > Duration(seconds=self._g("settle_duration")):
                if self._g("verify_distance") > 0.0:
                    self._verify_start = (
                        capture_point.copy(),
                        self._marker_pos.copy(),
                    )
                    self._locked = False  # re-enable vision to watch the marker move
                    self.get_logger().info(
                        f"Verifying capture: backing off {self._g('verify_distance')} m"
                    )
                    self._set_state(State.VERIFY)
                else:
                    self._set_state(State.HOLD)

        elif self._state == State.VERIFY:
            start_point, start_marker = self._verify_start
            goal = start_point - self._dock_dir * self._g("verify_distance")
            v = clamp_norm(
                self._g("kp_lin") * (goal - capture_point), self._g("verify_vel")
            )
            timed_out = (now - self._state_since) > Duration(
                seconds=self._g("verify_timeout")
            )
            if np.linalg.norm(goal - capture_point) < 0.02 or timed_out:
                moved_arm = capture_point - start_point
                moved_marker = (
                    self._marker_pos - start_marker
                    if self._marker_pos is not None
                    else np.full(3, np.nan)
                )
                back = -self._dock_dir
                along_arm = float(np.dot(moved_arm, back))
                along_marker = float(np.dot(moved_marker, back))
                ratio = along_marker / max(along_arm, 1e-6)
                captured = ratio > self._g("verify_follow_ratio")
                self.get_logger().info(
                    f"Capture check: arm moved {along_arm:.3f} m back, marker moved "
                    f"{along_marker:.3f} m (follow ratio {ratio:.2f}) -> "
                    f"{'CAPTURED' if captured else 'NOT CAPTURED'}"
                    f"{' [timeout]' if timed_out else ''}"
                )
                self._set_state(State.HOLD if captured else State.APPROACH)

        elif self._state == State.HOLD:
            self._set_state(
                State.RING_SEARCH if self._g("phase2_enabled") else State.DONE
            )

        return v, w

    def _align_motion(self, capture_point, R_f, approach, now):
        """Square the capture face onto the marker face, then hand over to DOCK.

        Without a usable face normal this degrades to APPROACH's behaviour (aim at the
        centroid) rather than stalling: a crooked capture still beats no capture, and
        `CaptureManager` reports the tilt either way.
        """
        timed_out = (now - self._state_since) > Duration(seconds=self._g("align_timeout"))
        if self._marker_axis is None:
            if timed_out:
                self.get_logger().warn(
                    "ALIGN never got a marker face normal (blob too small?) -> "
                    "docking on the centroid instead; expect a tilted capture"
                )
                self._dock_dir = normalize(self._marker_pos - capture_point)
                self._set_state(State.DOCK)
            return np.zeros(3), np.zeros(3)

        # The face normal points back at the camera, so the arm must come in along -n
        desired = -self._marker_axis
        goal = self._marker_pos + self._marker_axis * self._g("standoff")
        v, w = self._servo(
            capture_point, R_f, goal, desired, self._g("align_max_lin_vel")
        )

        ang_err = math.degrees(angle_between(approach, desired))
        offset = capture_point - self._marker_pos
        lat_err = float(
            np.linalg.norm(offset - np.dot(offset, self._marker_axis) * self._marker_axis)
        )
        within = ang_err < self._g("align_ang_tol_deg") and lat_err < self._g(
            "align_lat_tol"
        )
        if within:
            if self._in_tolerance_since is None:
                self._in_tolerance_since = now
            elif (now - self._in_tolerance_since) > Duration(
                seconds=self._g("align_hold_duration")
            ):
                self.get_logger().info(
                    f"Aligned to the marker face: {ang_err:.2f} deg tilt, "
                    f"{lat_err:.3f} m off axis -> docking"
                )
                self._dock_dir = desired
                self._set_state(State.DOCK)
        else:
            self._in_tolerance_since = None
            if timed_out:
                self.get_logger().warn(
                    f"ALIGN timed out at {ang_err:.2f} deg / {lat_err:.3f} m "
                    "-> docking anyway"
                )
                self._dock_dir = desired
                self._set_state(State.DOCK)
        return v, w

    ## Phase 2: peg-in-hole ##

    def _step_phase2(self, p_f, R_f, approach, R_b):
        """Returns (v, w, point) where `v` is wanted AT `point`, not at the flange."""
        now = self.get_clock().now()
        mep = self._lookup(self._g("debris_frame"))
        if mep is None:
            self.get_logger().warn(
                f"Waiting for TF: {self._g('debris_frame')} -- phase 2 needs the MEP "
                "pose to place the peg and its camera",
                throttle_duration_sec=5.0,
            )
            return np.zeros(3), np.zeros(3), p_f
        peg_tip, peg_dir = self._peg_pose(mep)
        seen_recently = self._last_ring_time is not None and (
            now - self._last_ring_time
        ) < Duration(seconds=self._g("ring_lost_timeout"))
        v = np.zeros(3)
        w = np.zeros(3)

        if self._state == State.RING_SEARCH:
            if seen_recently and self._ring_streak >= self._g("detections_to_start"):
                self.get_logger().info(
                    f"Thruster ring acquired at {np.round(self._hole_pos, 3)}, "
                    f"axis {np.round(self._hole_axis, 3)}"
                )
                self._set_state(State.RING_ALIGN)
            else:
                w = self._ring_scan_motion(peg_dir, now)

        elif self._state == State.RING_ALIGN:
            if not seen_recently:
                self._set_state(State.RING_SEARCH)
            else:
                goal = self._hole_pos + self._hole_axis * self._g("insert_standoff")
                v, w = self._insert_servo(peg_tip, peg_dir, goal)
                ang_err, lat_err, _ = self._insert_errors(peg_tip, peg_dir)
                axial_err = abs(
                    float(np.dot(goal - peg_tip, self._hole_axis))
                )
                within = (
                    ang_err < self._g("insert_ang_tol_deg")
                    and lat_err < self._g("insert_lat_tol")
                    and axial_err < 0.15
                )
                if within:
                    if self._in_tolerance_since is None:
                        self._in_tolerance_since = now
                    elif (now - self._in_tolerance_since) > Duration(
                        seconds=self._g("insert_hold_duration")
                    ):
                        self.get_logger().info(
                            f"Lined up on the hole: {ang_err:.2f} deg, "
                            f"{lat_err:.3f} m off axis -> inserting"
                        )
                        self._set_state(State.INSERT)
                        self._stall_ref = (now, peg_tip.copy())
                else:
                    self._in_tolerance_since = None

        elif self._state == State.INSERT:
            # Keep correcting while advancing: the lever from the flange to the tip
            # means small joint motions show up as lateral drift at the tip.
            v, w = self._insert_servo(peg_tip, peg_dir)
            ang_err, lat_err, depth = self._insert_errors(peg_tip, peg_dir)

            if depth >= self._g("insert_depth"):
                self.get_logger().info(
                    f"Peg inserted {depth:.3f} m past the nozzle mouth "
                    f"(target {self._g('insert_depth')} m) -> done"
                )
                self._set_state(State.INSERTED)
            elif self._stalled(now, peg_tip):
                self.get_logger().info(
                    f"Peg stopped advancing at {depth:.3f} m of insertion -- treating "
                    "this as contact with the converging nozzle bore -> done"
                )
                self._set_state(State.INSERTED)
            elif lat_err > 4.0 * self._g("insert_lat_tol"):
                self.get_logger().warn(
                    f"Drifted {lat_err:.3f} m off the hole axis while inserting "
                    "-> backing out to realign"
                )
                self._set_state(State.RING_ALIGN)

        elif self._state == State.INSERTED:
            self._set_state(State.DONE)

        return v, w, peg_tip

    def _peg_pose(self, mep) -> Tuple[np.ndarray, np.ndarray]:
        p_m, R_m = mep
        return p_m + R_m @ self._p_mep_peg, R_m @ self._d_mep_peg

    def _insert_errors(self, peg_tip, peg_dir) -> Tuple[float, float, float]:
        """(angle to the hole axis [deg], lateral offset [m], insertion depth [m])."""
        # `_hole_axis` points out of the hole, so the peg must point the other way
        ang = math.degrees(angle_between(peg_dir, -self._hole_axis))
        offset = peg_tip - self._hole_pos
        lat = float(np.linalg.norm(offset - np.dot(offset, self._hole_axis) * self._hole_axis))
        depth = -float(np.dot(offset, self._hole_axis))
        return ang, lat, depth

    def _insert_servo(self, peg_tip, peg_dir, goal=None):
        """Servo the PEG TIP, not the flange, onto the hole axis.

        With `goal` it flies to that point; without one it advances along the axis at
        `insert_vel` while correcting whatever lateral drift has crept in. The lateral
        term is measured against the axis LINE rather than a goal point on purpose --
        a goal that already lies on the axis has no across-axis component to correct
        towards, so deriving the correction from it would silently do nothing.
        """
        rot_err = np.cross(peg_dir, -self._hole_axis)
        w = clamp_norm(
            self._g("insert_kp_rot") * rot_err, self._g("insert_max_rot_vel")
        )
        if goal is not None:
            v = clamp_norm(
                self._g("insert_kp_lin") * (goal - peg_tip),
                self._g("insert_max_lin_vel"),
            )
            return v, w

        offset = peg_tip - self._hole_pos
        lateral = offset - np.dot(offset, self._hole_axis) * self._hole_axis
        v = -self._hole_axis * self._g("insert_vel") - clamp_norm(
            self._g("insert_kp_lin") * lateral, self._g("insert_max_lin_vel")
        )
        return v, w

    def _ring_scan_motion(self, peg_dir, now):
        """Hold still first, then sweep a slow cone about where the peg already points.

        The environment starts the MEP and the satellite collinear, so the ring is
        usually already in frame and the settle window alone finds it. The cone is the
        fallback, and it stays small on purpose: this is a 10 m body on the end of the
        arm, and a phase-1-sized sweep would fling it.
        """
        if (now - self._state_since) < Duration(seconds=self._g("ring_search_settle")):
            return np.zeros(3)
        elapsed = (now - self._state_since).nanoseconds * 1e-9
        phase = 2.0 * math.pi * elapsed / max(self._g("ring_scan_period"), 1e-3)
        side = normalize(np.cross(peg_dir, np.array([0.0, 0.0, 1.0])))
        if np.linalg.norm(side) < 1e-6:
            side = np.array([1.0, 0.0, 0.0])
        up = np.cross(peg_dir, side)
        tilt = math.radians(self._g("ring_scan_deg"))
        desired = (
            axis_angle(math.cos(phase) * side + math.sin(phase) * up, tilt) @ peg_dir
        )
        return clamp_norm(
            self._g("insert_kp_rot") * np.cross(peg_dir, desired),
            self._g("insert_max_rot_vel"),
        )

    def _stalled(self, now: Time, peg_tip: np.ndarray) -> bool:
        if self._stall_ref is None:
            self._stall_ref = (now, peg_tip.copy())
            return False
        since, ref = self._stall_ref
        if (now - since) < Duration(seconds=self._g("insert_stall_window")):
            return False
        travelled = float(np.linalg.norm(peg_tip - ref))
        self._stall_ref = (now, peg_tip.copy())
        return travelled < self._g("insert_stall_travel")

    ## Perception ##

    def _update_marker_perception(self, p_f: np.ndarray, R_f: np.ndarray):
        frame = self._cam_wrist.take_frame()
        if frame is None:
            return
        rgb, depth = frame
        mask = detect_color_blob(
            rgb, self._g("hsv_lower"), self._g("hsv_upper"),
            min_pixels=self._g("min_pixels"),
            depth=depth, max_depth=self._g("marker_max_depth"),
        )
        points_cam = backproject(
            mask, depth, self._cam_wrist.K,
            min_pixels=self._g("min_pixels"),
            max_depth=self._g("marker_max_depth"),
        )

        if self._g("publish_debug_image"):
            self._publish_debug(self._pub_debug, self._cam_wrist, rgb, mask)

        if self._locked:
            return
        if points_cam is None:
            self._detection_streak = 0
            return

        # Camera frame -> env frame
        R_cam = R_f @ self._R_l7_cam
        p_cam = p_f + R_f @ self._p_l7_cam
        points = points_cam @ R_cam.T + p_cam

        centroid, axis = self._marker_face(points_cam, R_cam, p_cam, len(points_cam))
        if centroid is None:
            # Long range: too few pixels to fit a plane. Fall back to the blob centroid,
            # pushed back along the viewing ray by the mean depth of a visible cylinder.
            centroid = points.mean(axis=0)
            ray = normalize(centroid - p_cam)
            centroid = centroid + ray * (math.pi * self._g("marker_radius") / 4.0)

        alpha = self._g("estimate_smoothing")
        if self._marker_pos is None or self._detection_streak == 0:
            self._marker_pos = centroid
        else:
            self._marker_pos = (1 - alpha) * self._marker_pos + alpha * centroid
        if axis is not None:
            self._marker_axis = (
                axis
                if self._marker_axis is None
                else normalize((1 - alpha) * self._marker_axis + alpha * axis)
            )

        self._detection_streak += 1
        self._last_detection_time = self.get_clock().now()
        self._publish_point(self._pub_marker, self._cam_wrist, self._marker_pos)

    def _marker_face(self, points_cam, R_cam, p_cam, n_pixels):
        """Centre and outward normal of the marker's flat face, or (None, None).

        The marker is a puck, so its cloud is a disc of front face plus a crescent of
        side wall. The side wall is always farther from the camera, so taking the near
        half isolates the face; RANSAC then shrugs off the mask's ragged edge.
        """
        if n_pixels < self._g("plane_fit_min_pixels"):
            return None, None
        near = near_face_points(
            points_cam, fraction=self._g("plane_fit_near_fraction")
        )
        fit = fit_plane_ransac(
            near,
            inlier_tol=self._g("plane_fit_inlier_tol"),
            orient_towards=np.zeros(3),  # the camera is the origin of this frame
        )
        if fit is None:
            return None, None
        face_centre_cam, normal_cam = fit
        normal = normalize(R_cam @ normal_cam)
        face_centre = R_cam @ face_centre_cam + p_cam
        # The capture point is the puck's centre, half a length in behind its face
        return face_centre - normal * self._g("marker_half_length"), normal

    def _update_ring_perception(self):
        frame = self._cam_mep.take_frame()
        if frame is None:
            return
        rgb, depth = frame
        mask = detect_color_blob(
            rgb, self._g("hsv_lower"), self._g("hsv_upper"),
            min_pixels=self._g("min_pixels"),
            depth=depth, max_depth=self._g("ring_max_depth"),
        )
        points_cam = backproject(
            mask, depth, self._cam_mep.K,
            min_pixels=self._g("min_pixels"),
            max_depth=self._g("ring_max_depth"),
        )
        if self._g("publish_debug_image"):
            self._publish_debug(self._pub_debug_mep, self._cam_mep, rgb, mask)
        if points_cam is None:
            self._ring_streak = 0
            return

        mep = self._lookup(self._g("debris_frame"))
        if mep is None:
            return
        p_m, R_m = mep
        R_cam = R_m @ self._R_mep_cam
        p_cam = p_m + R_m @ self._p_mep_cam

        fit = fit_circle_3d(points_cam)
        if fit is None:
            self._ring_streak = 0
            return
        centre_cam, normal_cam, radius = fit
        # A circle fit will happily return nonsense for a cloud that is not a ring
        if abs(radius - self._g("ring_radius_nominal")) > self._g(
            "ring_radius_tolerance"
        ):
            self._ring_streak = 0
            self.get_logger().warn(
                f"Ignoring a green blob that fits a circle of r={radius:.2f} m; the "
                f"thruster ring is r={self._g('ring_radius_nominal'):.2f} m",
                throttle_duration_sec=5.0,
            )
            return

        centre = R_cam @ centre_cam + p_cam
        axis = normalize(R_cam @ normal_cam)
        # Orient the axis out of the hole, i.e. back towards the camera that sees it
        if np.dot(p_cam - centre, axis) < 0.0:
            axis = -axis
        # The ring plane sits in front of the nozzle mouth by a known amount
        hole = centre - axis * self._g("ring_to_mouth")

        alpha = self._g("estimate_smoothing")
        if self._hole_pos is None or self._ring_streak == 0:
            self._hole_pos, self._hole_axis = hole, axis
        else:
            self._hole_pos = (1 - alpha) * self._hole_pos + alpha * hole
            self._hole_axis = normalize((1 - alpha) * self._hole_axis + alpha * axis)
        self._ring_streak += 1
        self._last_ring_time = self.get_clock().now()
        self._publish_point(self._pub_hole, self._cam_mep, self._hole_pos)

    def _publish_point(self, pub, cam: CameraStream, point: np.ndarray):
        msg = PointStamped()
        msg.header.frame_id = self._g("env_frame")
        msg.header.stamp = cam.rgb.header.stamp
        msg.point.x, msg.point.y, msg.point.z = map(float, point)
        pub.publish(msg)

    def _publish_debug(self, pub, cam: CameraStream, rgb: np.ndarray, mask: np.ndarray):
        import cv2

        overlay = rgb.copy()
        overlay[mask > 0] = (255, 0, 255)
        cv2.putText(
            overlay,
            f"{self._state.value} {cam.name}",
            (4, 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 0),
            1,
        )
        msg = Image()
        msg.header = cam.rgb.header
        msg.height, msg.width = overlay.shape[:2]
        msg.encoding = "rgb8"
        msg.step = 3 * msg.width
        msg.data = overlay.tobytes()
        pub.publish(msg)

    ## Control ##

    def _servo(
        self,
        capture_point: np.ndarray,
        R_f: np.ndarray,
        goal: np.ndarray,
        direction: np.ndarray,
        max_lin_vel: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        approach = self._g("approach_axis_sign") * R_f[:, 2]
        rot_err = np.cross(approach, direction)
        if np.dot(approach, direction) < 0.0:  # > 90 deg: cross vanishes near 180 deg
            rot_err = normalize(rot_err if np.linalg.norm(rot_err) > 1e-6 else R_f[:, 0])
        # Unlike a two-finger grasp there is no roll to line up: a magnet pad is
        # rotationally symmetric, so only the approach axis is constrained.
        w = clamp_norm(self._g("kp_rot") * rot_err, self._g("max_rot_vel"))
        v = clamp_norm(self._g("kp_lin") * (goal - capture_point), max_lin_vel)
        return v, w

    def _search_motion(
        self, capture_point: np.ndarray, approach: np.ndarray, R_b: np.ndarray, now: Time
    ) -> Tuple[np.ndarray, np.ndarray]:
        # 1) Recently lost: turn back towards the last known marker position
        if self._marker_pos is not None and (now - self._state_since) < Duration(
            seconds=self._g("reacquire_duration")
        ):
            desired = normalize(self._marker_pos - capture_point)
            w = clamp_norm(
                self._g("kp_rot") * self._look_error(approach, desired, R_b),
                self._g("max_rot_vel"),
            )
            return np.zeros(3), w

        # 2) Sweep search about the base axis
        base = self._lookup(self._g("base_frame"))
        base_pos = base[0] if base is not None else np.zeros(3)
        up = R_b[:, 2]
        if self._search_origin is None:
            self._search_origin = (capture_point.copy(), approach.copy())
            self._search_dir = self._search_first_dir
            self._search_pitch_idx = 0
            self.get_logger().info("Marker not visible -> sweep search")
        origin, look0 = self._search_origin

        # Progress is measured on the actual look direction, so the target never runs
        # away from an arm that cannot keep up (it simply leads by a fixed angle)
        yaw = self._signed_azimuth(look0, approach, up)
        limit = math.radians(self._g("search_sweep_deg"))
        margin = math.radians(2.0)
        if (self._search_dir > 0 and yaw >= limit - margin) or (
            self._search_dir < 0 and yaw <= -limit + margin
        ):
            self._search_dir = -self._search_dir
            if self._search_dir == self._search_first_dir:
                steps = self._g("search_pitch_steps_deg")
                self._search_pitch_idx = (self._search_pitch_idx + 1) % len(steps)
                self.get_logger().info(
                    f"Sweep done -> camera pitch {steps[self._search_pitch_idx]} deg"
                )
            else:
                self.get_logger().info(
                    f"Sweep reached {'+' if yaw > 0 else '-'}limit -> sweeping back"
                )
        target_yaw = float(
            np.clip(
                yaw + self._search_dir * math.radians(self._g("search_lead_deg")),
                -limit,
                limit,
            )
        )

        R_yaw = axis_angle(up, target_yaw)
        goal = base_pos + R_yaw @ (origin - base_pos)
        side = np.cross(look0, up)
        side = normalize(side if np.linalg.norm(side) > 1e-3 else R_b[:, 0])
        pitch = self._g("search_pitch_steps_deg")[self._search_pitch_idx]
        desired = R_yaw @ axis_angle(side, math.radians(pitch)) @ look0

        v = clamp_norm(self._g("kp_lin") * (goal - capture_point), self._g("max_lin_vel"))
        w = clamp_norm(
            self._g("kp_rot") * self._look_error(approach, desired, R_b),
            self._g("search_max_rot_vel"),
        )
        return v, w

    @staticmethod
    def _signed_azimuth(a: np.ndarray, b: np.ndarray, up: np.ndarray) -> float:
        a_p = a - np.dot(a, up) * up
        b_p = b - np.dot(b, up) * up
        return math.atan2(float(np.dot(up, np.cross(a_p, b_p))), float(np.dot(a_p, b_p)))

    @staticmethod
    def _look_error(approach: np.ndarray, desired: np.ndarray, R_b: np.ndarray) -> np.ndarray:
        err = np.cross(approach, desired)
        # Near 180 deg the cross product vanishes -> turn about the base axis
        if np.dot(approach, desired) < 0.0 and np.linalg.norm(err) < 0.5:
            err = R_b[:, 2]
        return err

    def _publish_command(
        self,
        v_at_point: np.ndarray,
        w: np.ndarray,
        point: np.ndarray,
        R_b: np.ndarray,
        p_f: np.ndarray,
        approach: np.ndarray,
    ):
        """Command a twist that realises `v_at_point` AT `point`.

        The IK rotates about its control point, which swings every other point on the
        arm -- and in phase 2 `point` is the peg tip, ~11 m out, where that swing
        dominates the motion. Compensating here is what lets the callers reason about
        the point they actually care about.
        """
        control_point = p_f + approach * self._g("ik_control_point_link7")
        v_ctrl = v_at_point - np.cross(w, point - control_point)

        lin = R_b.T @ v_ctrl / self._g("ik_lin_gain")
        ang = R_b.T @ w / self._g("ik_rot_gain")
        max_action = self._g("max_action")
        lin, ang = clamp_norm(lin, max_action), clamp_norm(ang, max_action)

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = map(float, lin)
        msg.angular.x, msg.angular.y, msg.angular.z = map(float, ang)
        self._pub_twist.publish(msg)

    ## Utils ##

    def _warn_missing_frames(self, link_missing: bool, base_missing: bool):
        """Say which frame is holding everything up, instead of stalling silently.

        A missing frame makes `_step` return before it ever touches the camera, so
        the debug image stays empty too -- the symptom gives no hint on its own.
        """
        missing = []
        if link_missing:
            missing.append(self._g("link_frame"))
        if base_missing:
            missing.append(self._g("base_frame"))
        self.get_logger().warn(
            f"Waiting for TF: {', '.join(missing)} (relative to {self._g('env_frame')}). "
            "Is the simulation playing? Check with: ros2 run tf2_tools view_frames",
            throttle_duration_sec=5.0,
        )

    def _lookup(self, frame: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        try:
            tf = self._tf_buffer.lookup_transform(self._g("env_frame"), frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), quat_to_matrix(q.x, q.y, q.z, q.w)

    def _camera_pos(self, p_f: np.ndarray, R_f: np.ndarray) -> np.ndarray:
        return p_f + R_f @ self._p_l7_cam

    def _set_state(self, state: State):
        if state in (State.SEARCH, State.APPROACH, State.ALIGN):
            # Re-enable vision updates (the estimate is only frozen during DOCK)
            self._locked = False
        if state == State.SEARCH:
            self._search_origin = None
            self._detection_streak = 0
        self._in_tolerance_since = None
        self._stall_ref = None
        self.get_logger().info(f"{self._state.value} -> {state.value}")
        self._state = state
        self._state_since = self.get_clock().now()

    def stop(self):
        # The IK action latches the last command -> leave the arm with a zero twist
        if rclpy.ok():
            self._pub_twist.publish(Twist())


def main():
    rclpy.init()
    node = MepSuctionCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
