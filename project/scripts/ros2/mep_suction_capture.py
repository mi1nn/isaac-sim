#!/usr/bin/env python3
"""Vision-guided magnet capture of the MEP with a gripper-less Canadarm3.

Runs outside of Isaac Sim (system ROS 2), next to:
    srb agent ros --interface ros --env debris_capture_visual \
        --kit_args "--ext-folder /home/rokey/isaac-sim/apps --enable isaacsim.exp.base" \
        env.robot=canadarm3

This is the suction/magnet counterpart of `mep_handle_grasp.py`. The detection and
servo maths are the same; what differs is everything that used to involve fingers:

  * No gripper, and therefore no `srb/env0/end_effector` frame. The ROS interface
    broadcasts one frame per scene asset and one per articulation link, but skips
    FrameTransformer sensors such as `tf_end_effector`, and `scene.end_effector` is
    None when the robot carries no tool. Everything here is therefore anchored on
    `srb/env0/robot/canadarm3_large_7`, the arm's last link, whose pose comes
    straight from PhysX. Link 7 reaches along its own -Z, which is where the flange
    (0.44 m), the capture cylinder centre (0.64 m) and the wrist camera (0.9 m) all
    sit. The sign is a parameter (`approach_axis_sign`) rather than a buried
    constant, because getting it wrong drives the arm backwards.

  * Nothing to command. `CaptureManager` (srb/tasks/manipulation/debris_capture/
    capture.py) watches the distance between the capture cylinder's centre and the
    MEP marker's centre every physics step, and welds the two with a fixed joint as
    soon as that distance drops below `CaptureCfg.distance_threshold` (0.3 m by
    default). There is no "close the gripper" topic: the job is purely to fly the
    capture point onto the marker and then stop pushing.

  * No `srb/env0/mep` frame. This branch does not set `mep_ros_frames`, so the MEP
    pose is not published over TF and capture cannot be confirmed by reading it.
    VERIFY instead backs the arm off a few centimetres and checks whether the
    *detected marker* follows: welded to the arm it moves with the TCP, free it
    stays where it was. That is the same follow-ratio idea the grasp script used,
    measured with the camera rather than with TF.

Frames (all published by the SRB ROS interface)
    srb/env0                            env frame (all control math happens here)
    srb/env0/robot                      Canadarm3 base (IK deltas are in this frame)
    srb/env0/robot/canadarm3_large_7    last link (-Z = the direction the arm reaches)

NOTE: The TF of "srb/env0/cam_wrist" is NOT used, because Isaac Lab reads the camera
pose from USD, which does not follow the moving articulation. The camera pose is
derived from the link-7 TF and the known camera mount instead.
"""

import enum
import math
from typing import Optional, Tuple

import cv2
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

QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class State(enum.Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    DOCK = "DOCK"
    SETTLE = "SETTLE"
    VERIFY = "VERIFY"
    HOLD = "HOLD"
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
    return rot_z(rpy_deg[2]) @ rot_y(rpy_deg[1]) @ rot_x(rpy_deg[0])


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def clamp_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = np.linalg.norm(v)
    return v * (max_norm / n) if n > max_norm else v


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    return math.acos(float(np.clip(np.dot(normalize(a), normalize(b)), -1.0, 1.0)))


## Node ##


class MepSuctionCapture(Node):
    def __init__(self):
        super().__init__("mep_suction_capture")

        ## Parameters
        p = self.declare_parameter
        # Topics & frames
        p("env_ns", "/srb/env0")
        p("camera_name", "cam_wrist")
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

        # Detection (OpenCV HSV: H in [0, 180])
        p("hsv_lower", [45, 120, 60])
        p("hsv_upper", [75, 255, 255])
        p("min_pixels", 12)
        p("max_depth", 30.0)
        p("estimate_smoothing", 0.5)  # EMA weight of a new measurement
        # Depth only sees the front half of the marker cylinder, whose mean lies
        # pi*r/4 in front of the axis -> push the estimate back along the viewing ray.
        # The marker measures ~0.165 m across after TaskCfg.debris_marker_xform, i.e.
        # a radius of ~0.082 m, scaled by the debris spawn scale (1.1).
        p("marker_radius", 0.09)

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

        # Debug
        p("publish_debug_image", True)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._g = g

        ## Fixed transforms, expressed directly in the link-7 frame.
        R_gl_ros = np.diag([1.0, -1.0, -1.0])  # OpenGL camera -> ROS optical frame
        self._R_l7_cam = rpy_to_matrix(g("camera_rpy_link7")) @ R_gl_ros
        self._p_l7_cam = np.array(g("camera_pos_link7"))
        self.get_logger().info(
            f"Camera in link 7: p={np.round(self._p_l7_cam, 3)} "
            f"optical axis={np.round(self._R_l7_cam[:, 2], 3)} | "
            f"capture point {g('capture_offset_link7')} m, "
            f"IK control point {g('ik_control_point_link7')} m along the reach axis"
        )

        ## ROS interfaces
        ns = g("env_ns").rstrip("/")
        cam = g("camera_name")
        self.create_subscription(Image, f"{ns}/{cam}/image_rgb", self._cb_rgb, QOS)
        self.create_subscription(Image, f"{ns}/{cam}/image_depth", self._cb_depth, QOS)
        self.create_subscription(CameraInfo, f"{ns}/{cam}/camera_info", self._cb_info, QOS)
        self._pub_twist = self.create_publisher(Twist, g("twist_topic"), QOS)
        self._pub_state = self.create_publisher(String, "~/state", QOS)
        self._pub_marker = self.create_publisher(PointStamped, "~/marker", QOS)
        self._pub_debug = self.create_publisher(Image, "~/debug_image", QOS)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        ## State
        self._rgb: Optional[Image] = None
        self._depth: Optional[Image] = None
        self._K: Optional[np.ndarray] = None
        self._last_processed_stamp: Optional[Tuple[int, int]] = None
        self._state = State.SEARCH
        self._state_since = self.get_clock().now()
        self._marker_pos: Optional[np.ndarray] = None
        self._last_detection_time: Optional[Time] = None
        self._detection_streak = 0
        self._locked = False
        self._dock_dir: Optional[np.ndarray] = None
        self._verify_start: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._search_origin: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._search_first_dir = math.copysign(1.0, g("search_first_dir"))
        self._search_dir = self._search_first_dir
        self._search_pitch_idx = 0

        self.create_timer(1.0 / g("control_rate"), self._step)
        self.get_logger().info(
            "MEP suction capture node started (state: SEARCH). Capture is automatic: "
            f"CaptureManager welds the MEP once the capture point is within "
            f"{g('capture_distance_threshold')} m of the marker."
        )

    ## Callbacks ##

    def _cb_rgb(self, msg: Image):
        self._rgb = msg

    def _cb_depth(self, msg: Image):
        self._depth = msg

    def _cb_info(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    ## Main loop ##

    def _step(self):
        link = self._lookup(self._g("link_frame"))
        base = self._lookup(self._g("base_frame"))
        if link is None or base is None:
            self._warn_missing_frames(link is None, base is None)
            return
        p_f, R_f = link
        _, R_b = base

        self._update_perception(p_f, R_f)

        now = self.get_clock().now()
        detected_recently = self._last_detection_time is not None and (
            now - self._last_detection_time
        ) < Duration(seconds=self._g("lost_timeout"))

        v = np.zeros(3)
        w = np.zeros(3)

        # The reaching direction of the flange, and the capture cylinder's centre.
        approach = self._g("approach_axis_sign") * R_f[:, 2]
        capture_point = p_f + approach * self._g("capture_offset_link7")

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
                    capture_point, p_f, R_f, goal, d, self._g("max_lin_vel")
                )
                pos_err = np.linalg.norm(goal - capture_point)
                ang_err = math.degrees(angle_between(approach, d))
                if pos_err < self._g("approach_pos_tol") and ang_err < self._g(
                    "approach_ang_tol_deg"
                ):
                    self._dock_dir = d
                    self._set_state(State.DOCK)

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
            v, w = self._servo(
                capture_point, p_f, R_f, goal, d, self._g("dock_lin_vel")
            )
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
            self._set_state(State.DONE)

        self._publish_command(v, w, R_b, p_f, R_f, approach)
        self._pub_state.publish(String(data=self._state.value))

    ## Perception ##

    def _update_perception(self, p_f: np.ndarray, R_f: np.ndarray):
        if self._rgb is None or self._depth is None or self._K is None:
            return
        stamp = (self._rgb.header.stamp.sec, self._rgb.header.stamp.nanosec)
        if stamp == self._last_processed_stamp:
            return
        self._last_processed_stamp = stamp

        rgb = self._image_to_numpy(self._rgb)
        depth = self._image_to_numpy(self._depth)
        if rgb is None or depth is None:
            return
        mask = self._detect(rgb)
        points_cam = self._backproject(mask, depth)

        if self._g("publish_debug_image"):
            self._publish_debug(rgb, mask)

        if self._locked:
            return
        if points_cam is None:
            self._detection_streak = 0
            return

        # Camera frame -> flange frame -> env frame
        R_cam = R_f @ self._R_l7_cam
        p_cam = p_f + R_f @ self._p_l7_cam
        points = points_cam @ R_cam.T + p_cam

        centroid = points.mean(axis=0)
        ray = normalize(centroid - p_cam)
        centroid = centroid + ray * (math.pi * self._g("marker_radius") / 4.0)

        alpha = self._g("estimate_smoothing")
        if self._marker_pos is None or self._detection_streak == 0:
            self._marker_pos = centroid
        else:
            self._marker_pos = (1 - alpha) * self._marker_pos + alpha * centroid

        self._detection_streak += 1
        self._last_detection_time = self.get_clock().now()

        msg = PointStamped()
        msg.header.frame_id = self._g("env_frame")
        msg.header.stamp = self._rgb.header.stamp
        msg.point.x, msg.point.y, msg.point.z = map(float, self._marker_pos)
        self._pub_marker.publish(msg)

    def _detect(self, rgb: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(
            hsv, np.array(self._g("hsv_lower")), np.array(self._g("hsv_upper"))
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if n <= 1:
            return np.zeros_like(mask)
        largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        return (labels == largest).astype(np.uint8) * 255

    def _backproject(self, mask: np.ndarray, depth: np.ndarray) -> Optional[np.ndarray]:
        vs, us = np.nonzero(mask)
        if len(us) < self._g("min_pixels"):
            return None
        z = depth[vs, us].astype(np.float64)
        valid = np.isfinite(z) & (z > 0.0) & (z < self._g("max_depth"))
        if valid.sum() < self._g("min_pixels"):
            return None
        us, vs, z = us[valid], vs[valid], z[valid]
        fx, fy, cx, cy = self._K[0, 0], self._K[1, 1], self._K[0, 2], self._K[1, 2]
        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy
        return np.stack([x, y, z], axis=1)

    @staticmethod
    def _image_to_numpy(msg: Image) -> Optional[np.ndarray]:
        if msg.encoding in ("rgb8", "rgba8"):
            ch = 3 if msg.encoding == "rgb8" else 4
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
            return np.ascontiguousarray(img[:, :, :3])
        if msg.encoding == "32FC1":
            return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return None

    def _publish_debug(self, rgb: np.ndarray, mask: np.ndarray):
        overlay = rgb.copy()
        overlay[mask > 0] = (255, 0, 255)
        cv2.putText(
            overlay, self._state.value, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1
        )
        msg = Image()
        msg.header = self._rgb.header
        msg.height, msg.width = overlay.shape[:2]
        msg.encoding = "rgb8"
        msg.step = 3 * msg.width
        msg.data = overlay.tobytes()
        self._pub_debug.publish(msg)

    ## Control ##

    def _servo(
        self,
        capture_point: np.ndarray,
        p_f: np.ndarray,
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

        R_yaw = self._axis_angle(up, target_yaw)
        goal = base_pos + R_yaw @ (origin - base_pos)
        side = np.cross(look0, up)
        side = normalize(side if np.linalg.norm(side) > 1e-3 else R_b[:, 0])
        pitch = self._g("search_pitch_steps_deg")[self._search_pitch_idx]
        desired = R_yaw @ self._axis_angle(side, math.radians(pitch)) @ look0

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

    @staticmethod
    def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
        a = normalize(axis)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K

    def _publish_command(
        self,
        v_capture: np.ndarray,
        w: np.ndarray,
        R_b: np.ndarray,
        p_f: np.ndarray,
        R_f: np.ndarray,
        approach: np.ndarray,
    ):
        # The IK rotates about its control point, which swings the capture point ->
        # compensate so the commanded velocity is the one the capture point sees.
        control_point = p_f + approach * self._g("ik_control_point_link7")
        capture_point = p_f + approach * self._g("capture_offset_link7")
        v_ctrl = v_capture - np.cross(w, capture_point - control_point)

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
        if state in (State.SEARCH, State.APPROACH):
            # Re-enable vision updates (the estimate is only frozen during DOCK)
            self._locked = False
        if state == State.SEARCH:
            self._search_origin = None
            self._detection_streak = 0
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
