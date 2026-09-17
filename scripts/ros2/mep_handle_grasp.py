#!/usr/bin/env python3
"""Vision-guided grasp of the MEP handle with Canadarm3 + Kinova300 (large).

Runs outside of Isaac Sim (system ROS 2), next to:
    srb agent ros --env debris_capture_visual env.robot=canadarm3+kinova300_large ...

Pipeline
    1. Wrist camera RGB  -> colour mask of the handle marker (green by default)
    2. Wrist camera depth -> 3D handle points -> centroid + pin axis (PCA)
    3. Gripper pose (TF) -> camera pose -> handle pose in the env frame
    4. State machine: SEARCH (arm orbits to look around) -> APPROACH -> FINAL -> CLOSE -> HOLD (-> RETREAT -> DONE)
    5. Twist on the relative differential-IK action topic, Bool on the gripper topic

Frames (all published by the SRB ROS interface)
    srb/env0               env frame (all control math happens here)
    srb/env0/robot         Canadarm3 base (IK deltas are expressed in this frame)
    srb/env0/end_effector  Kinova300 base (fingers point along its -Z axis)

NOTE: The TF of "srb/env0/cam_wrist" is NOT used, because Isaac Lab reads the camera
pose from USD, which does not follow the moving articulation. The camera pose is
derived from the gripper TF and the known camera mount instead.
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
from std_msgs.msg import Bool, String
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
    FINAL = "FINAL"
    CLOSE = "CLOSE"
    HOLD = "HOLD"
    RETREAT = "RETREAT"
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


class MepHandleGrasp(Node):
    def __init__(self):
        super().__init__("mep_handle_grasp")

        ## Parameters
        p = self.declare_parameter
        # Topics & frames
        p("env_ns", "/srb/env0")
        p("camera_name", "cam_wrist")
        p("twist_topic", "/srb/env0/robot/robot/differential_inverse_kinematics")
        p("gripper_topic", "/srb/env0/end_effector/end_effector/binary_joint_position")
        p("env_frame", "srb/env0")
        p("base_frame", "srb/env0/robot")
        p("gripper_frame", "srb/env0/end_effector")
        # Kinematics (see srb/assets/robot/manipulation/canadarm3.py & kinova_gripper.py)
        p("flange_pos_link7", [0.0, 0.0, -0.44])
        p("flange_rpy_link7", [0.0, 180.0, 0.0])
        p("mount_rpy_gripper", [180.0, 0.0, 0.0])
        p("camera_pos_link7", [0.0, 0.0, -1.1])
        p("camera_rpy_link7", [0.0, 0.0, -90.0])  # USD/OpenGL camera convention
        p("tcp_offset", 0.62)  # grasp point along the gripper -Z axis [m]
        # Detection (OpenCV HSV: H in [0, 180])
        p("hsv_lower", [45, 120, 60])
        p("hsv_upper", [75, 255, 255])
        p("min_pixels", 12)
        p("max_depth", 30.0)
        p("estimate_smoothing", 0.3)  # EMA weight of a new measurement
        # Control
        p("control_rate", 20.0)
        p("ik_lin_gain", 0.17)  # [m/s] of motion per unit of IK action
        p("ik_rot_gain", 0.17)  # [rad/s] of motion per unit of IK action
        p("ik_control_point", 0.01)  # IK body offset along gripper -Z [m]
        p("max_action", 1.0)
        p("kp_lin", 0.6)
        p("kp_rot", 0.8)
        p("max_lin_vel", 0.12)
        p("max_rot_vel", 0.12)
        p("final_lin_vel", 0.04)
        p("roll_weight", 0.5)
        p("standoff", 0.6)
        p("approach_pos_tol", 0.05)
        p("approach_ang_tol_deg", 6.0)
        p("lock_distance", 0.9)  # stop updating the estimate when closer than this
        p("grasp_pos_tol", 0.03)
        p("close_duration", 3.0)
        # Grasp check: pull back with the gripper closed and compare the MEP motion
        p("retreat_distance", 0.0)  # [m], 0 disables the check
        p("retreat_vel", 0.05)
        p("retreat_timeout", 90.0)
        p("hold_duration", 3.0)
        p("mep_frame", "srb/env0/mep")
        p("lost_timeout", 2.0)
        p("detections_to_start", 3)
        # Search
        # The arm orbits 360 deg about the base axis (moving the TCP along the arc) while
        # looking outwards; each full orbit uses the next pitch of the camera
        p("search_rate", 0.1)  # [rad/s] orbit rate about the base axis
        p("search_pitch_steps_deg", [0.0, 30.0, -30.0])
        p("search_track_tol", 0.25)  # [m] the orbit only advances while tracking well
        p("search_track_ang_deg", 20.0)
        p("reacquire_duration", 10.0)  # [s] look at the last known handle pose first
        # Debug
        p("publish_debug_image", True)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._g = g

        ## Fixed transforms
        R_l7_g = rpy_to_matrix(g("flange_rpy_link7")) @ rpy_to_matrix(g("mount_rpy_gripper"))
        p_l7_g = np.array(g("flange_pos_link7"))
        R_gl_ros = np.diag([1.0, -1.0, -1.0])  # OpenGL camera -> ROS optical frame
        R_l7_cam = rpy_to_matrix(g("camera_rpy_link7")) @ R_gl_ros
        self._R_g_cam = R_l7_g.T @ R_l7_cam
        self._p_g_cam = R_l7_g.T @ (np.array(g("camera_pos_link7")) - p_l7_g)
        self.get_logger().info(
            f"Camera in gripper frame: p={np.round(self._p_g_cam, 3)} "
            f"optical axis={np.round(self._R_g_cam[:, 2], 3)}"
        )

        ## ROS interfaces
        ns = g("env_ns").rstrip("/")
        cam = g("camera_name")
        self.create_subscription(Image, f"{ns}/{cam}/image_rgb", self._cb_rgb, QOS)
        self.create_subscription(Image, f"{ns}/{cam}/image_depth", self._cb_depth, QOS)
        self.create_subscription(CameraInfo, f"{ns}/{cam}/camera_info", self._cb_info, QOS)
        self._pub_twist = self.create_publisher(Twist, g("twist_topic"), QOS)
        self._pub_gripper = self.create_publisher(Bool, g("gripper_topic"), QOS)
        self._pub_state = self.create_publisher(String, "~/state", QOS)
        self._pub_handle = self.create_publisher(PointStamped, "~/handle", QOS)
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
        self._handle_pos: Optional[np.ndarray] = None
        self._handle_axis: Optional[np.ndarray] = None
        self._last_detection_time: Optional[Time] = None
        self._detection_streak = 0
        self._locked = False
        self._final_dir: Optional[np.ndarray] = None
        self._retreat_start: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._search_origin: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._search_yaw = 0.0
        self._search_pitch_idx = 0

        self.create_timer(1.0 / g("control_rate"), self._step)
        self.get_logger().info("MEP handle grasp node started (state: SEARCH)")

    ## Callbacks ##

    def _cb_rgb(self, msg: Image):
        self._rgb = msg

    def _cb_depth(self, msg: Image):
        self._depth = msg

    def _cb_info(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    ## Main loop ##

    def _step(self):
        gripper = self._lookup(self._g("gripper_frame"))
        base = self._lookup(self._g("base_frame"))
        if gripper is None or base is None:
            return
        p_g, R_g = gripper
        _, R_b = base

        self._update_perception(p_g, R_g)

        now = self.get_clock().now()
        detected_recently = self._last_detection_time is not None and (
            now - self._last_detection_time
        ) < Duration(seconds=self._g("lost_timeout"))

        v = np.zeros(3)
        w = np.zeros(3)
        close_gripper = self._state in (State.CLOSE, State.HOLD, State.RETREAT, State.DONE)

        z_g = R_g[:, 2]
        approach = -z_g
        tcp = p_g + approach * self._g("tcp_offset")

        if self._state == State.SEARCH:
            if self._handle_pos is not None and self._detection_streak >= self._g(
                "detections_to_start"
            ):
                self._set_state(State.APPROACH)
            else:
                v, w = self._search_motion(tcp, approach, R_b, now)

        elif self._state == State.APPROACH:
            if not detected_recently:
                self._set_state(State.SEARCH)
            else:
                d = self._approach_direction(tcp)
                goal = self._handle_pos - d * self._g("standoff")
                v, w = self._servo(tcp, p_g, R_g, goal, d, self._g("max_lin_vel"))
                pos_err = np.linalg.norm(goal - tcp)
                ang_err = math.degrees(angle_between(approach, d))
                if pos_err < self._g("approach_pos_tol") and ang_err < self._g(
                    "approach_ang_tol_deg"
                ):
                    self._final_dir = d
                    self._set_state(State.FINAL)

        elif self._state == State.FINAL:
            d = self._final_dir
            if not self._locked and np.linalg.norm(
                self._handle_pos - self._camera_pos(p_g, R_g)
            ) < self._g("lock_distance"):
                self._locked = True
                self.get_logger().info(
                    f"Handle estimate locked at {np.round(self._handle_pos, 3)}"
                )
            goal = self._handle_pos
            v, w = self._servo(tcp, p_g, R_g, goal, d, self._g("final_lin_vel"))
            if np.linalg.norm(goal - tcp) < self._g("grasp_pos_tol"):
                self._set_state(State.CLOSE)

        elif self._state == State.CLOSE:
            if (now - self._state_since) > Duration(seconds=self._g("close_duration")):
                self._set_state(State.HOLD)

        elif self._state == State.HOLD:
            mep = self._lookup(self._g("mep_frame"))
            if (
                self._g("retreat_distance") > 0.0
                and mep is not None
                and (now - self._state_since) > Duration(seconds=self._g("hold_duration"))
            ):
                self._retreat_start = (tcp.copy(), mep[0].copy())
                self.get_logger().info(
                    f"Grasp check: retreating {self._g('retreat_distance')} m "
                    f"(tcp={np.round(tcp, 3)}, mep={np.round(mep[0], 3)})"
                )
                self._set_state(State.RETREAT)

        elif self._state == State.RETREAT:
            tcp0, mep0 = self._retreat_start
            goal = tcp0 - self._final_dir * self._g("retreat_distance")
            v = clamp_norm(self._g("kp_lin") * (goal - tcp), self._g("retreat_vel"))
            timed_out = (now - self._state_since) > Duration(
                seconds=self._g("retreat_timeout")
            )
            if np.linalg.norm(goal - tcp) < self._g("grasp_pos_tol") or timed_out:
                mep = self._lookup(self._g("mep_frame"))
                tcp_moved = tcp - tcp0
                mep_moved = mep[0] - mep0 if mep is not None else np.full(3, np.nan)
                along = float(np.dot(mep_moved, -self._final_dir))
                ratio = along / max(float(np.dot(tcp_moved, -self._final_dir)), 1e-6)
                self.get_logger().info(
                    f"Grasp check result: tcp moved {np.linalg.norm(tcp_moved):.3f} m, "
                    f"mep moved {np.linalg.norm(mep_moved):.3f} m "
                    f"({along:.3f} m along the retreat, follow ratio {ratio:.2f}) -> "
                    f"{'HELD' if ratio > 0.8 else 'SLIPPED / NOT HELD'}"
                    f"{' [timeout]' if timed_out else ''}"
                )
                self._set_state(State.DONE)

        self._publish_command(v, w, R_b, p_g, R_g, close_gripper)
        self._pub_state.publish(String(data=self._state.value))

    ## Perception ##

    def _update_perception(self, p_g: np.ndarray, R_g: np.ndarray):
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

        # Camera frame -> gripper frame -> env frame
        R_cam = R_g @ self._R_g_cam
        p_cam = p_g + R_g @ self._p_g_cam
        points = points_cam @ R_cam.T + p_cam
        centroid = points.mean(axis=0)

        alpha = self._g("estimate_smoothing")
        if self._handle_pos is None or self._detection_streak == 0:
            self._handle_pos = centroid
        else:
            self._handle_pos = (1 - alpha) * self._handle_pos + alpha * centroid
        axis = self._pin_axis(points)
        if axis is not None:
            if self._handle_axis is not None and np.dot(axis, self._handle_axis) < 0:
                axis = -axis
            self._handle_axis = (
                axis
                if self._handle_axis is None
                else normalize((1 - alpha) * self._handle_axis + alpha * axis)
            )

        self._detection_streak += 1
        self._last_detection_time = self.get_clock().now()

        msg = PointStamped()
        msg.header.frame_id = self._g("env_frame")
        msg.header.stamp = self._rgb.header.stamp
        msg.point.x, msg.point.y, msg.point.z = map(float, self._handle_pos)
        self._pub_handle.publish(msg)

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
    def _pin_axis(points: np.ndarray) -> Optional[np.ndarray]:
        if len(points) < 30:
            return None
        centered = points - points.mean(axis=0)
        eigvals, eigvecs = np.linalg.eigh(np.cov(centered.T))
        # The pin is ~0.57 m long and 0.15 m wide -> require a clearly elongated cloud
        if eigvals[2] < 4.0 * max(eigvals[1], 1e-9):
            return None
        return normalize(eigvecs[:, 2])

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

    def _approach_direction(self, tcp: np.ndarray) -> np.ndarray:
        to_handle = self._handle_pos - tcp
        if self._handle_axis is not None:
            # Side grasp: approach perpendicular to the pin
            perp = to_handle - np.dot(to_handle, self._handle_axis) * self._handle_axis
            if np.linalg.norm(perp) > 1e-3:
                return normalize(perp)
        return normalize(to_handle)

    def _servo(
        self,
        tcp: np.ndarray,
        p_g: np.ndarray,
        R_g: np.ndarray,
        goal: np.ndarray,
        direction: np.ndarray,
        max_lin_vel: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        approach = -R_g[:, 2]
        rot_err = np.cross(approach, direction)
        if np.dot(approach, direction) < 0.0:  # > 90 deg: cross product vanishes near 180 deg
            rot_err = normalize(rot_err if np.linalg.norm(rot_err) > 1e-6 else R_g[:, 0])
        if self._handle_axis is not None:
            # Fingers close along gripper Y -> the pin must lie along gripper X
            x_g = R_g[:, 0]
            pin = self._handle_axis if np.dot(x_g, self._handle_axis) >= 0 else -self._handle_axis
            rot_err = rot_err + self._g("roll_weight") * np.cross(x_g, pin)
        w = clamp_norm(self._g("kp_rot") * rot_err, self._g("max_rot_vel"))
        v = clamp_norm(self._g("kp_lin") * (goal - tcp), max_lin_vel)
        return v, w

    def _search_motion(
        self, tcp: np.ndarray, approach: np.ndarray, R_b: np.ndarray, now: Time
    ) -> Tuple[np.ndarray, np.ndarray]:
        # 1) Recently lost: turn back towards the last known handle position
        if self._handle_pos is not None and (now - self._state_since) < Duration(
            seconds=self._g("reacquire_duration")
        ):
            desired = normalize(self._handle_pos - tcp)
            w = clamp_norm(
                self._g("kp_rot") * self._look_error(approach, desired, R_b),
                self._g("max_rot_vel"),
            )
            return np.zeros(3), w

        # 2) Orbit search about the base axis
        base = self._lookup(self._g("base_frame"))
        base_pos = base[0] if base is not None else np.zeros(3)
        up = R_b[:, 2]
        if self._search_origin is None:
            self._search_origin = (tcp.copy(), approach.copy())
            self._search_yaw = 0.0
            self._search_pitch_idx = 0
            self.get_logger().info("Handle not visible -> orbit search")
        tcp0, look0 = self._search_origin

        R_yaw = self._axis_angle(up, self._search_yaw)
        goal = base_pos + R_yaw @ (tcp0 - base_pos)
        side = np.cross(look0, up)
        side = normalize(side if np.linalg.norm(side) > 1e-3 else R_b[:, 0])
        pitch = self._g("search_pitch_steps_deg")[self._search_pitch_idx]
        desired = R_yaw @ self._axis_angle(side, math.radians(pitch)) @ look0

        # Advance along the orbit only while the arm keeps up with it
        tracking = np.linalg.norm(goal - tcp) < self._g("search_track_tol") and math.degrees(
            angle_between(approach, desired)
        ) < self._g("search_track_ang_deg")
        if tracking:
            self._search_yaw += self._g("search_rate") / self._g("control_rate")
            if self._search_yaw >= 2.0 * math.pi:
                self._search_yaw -= 2.0 * math.pi
                steps = self._g("search_pitch_steps_deg")
                self._search_pitch_idx = (self._search_pitch_idx + 1) % len(steps)
                self.get_logger().info(
                    f"Orbit done -> camera pitch {steps[self._search_pitch_idx]} deg"
                )

        v = clamp_norm(self._g("kp_lin") * (goal - tcp), self._g("max_lin_vel"))
        w = clamp_norm(
            self._g("kp_rot") * self._look_error(approach, desired, R_b),
            self._g("max_rot_vel"),
        )
        return v, w

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
        v_tcp: np.ndarray,
        w: np.ndarray,
        R_b: np.ndarray,
        p_g: np.ndarray,
        R_g: np.ndarray,
        close_gripper: bool,
    ):
        # The IK rotates about its control point, which swings the TCP -> compensate
        control_point = p_g - R_g[:, 2] * self._g("ik_control_point")
        tcp = p_g - R_g[:, 2] * self._g("tcp_offset")
        v_ctrl = v_tcp - np.cross(w, tcp - control_point)

        lin = R_b.T @ v_ctrl / self._g("ik_lin_gain")
        ang = R_b.T @ w / self._g("ik_rot_gain")
        max_action = self._g("max_action")
        lin, ang = clamp_norm(lin, max_action), clamp_norm(ang, max_action)

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = map(float, lin)
        msg.angular.x, msg.angular.y, msg.angular.z = map(float, ang)
        self._pub_twist.publish(msg)
        self._pub_gripper.publish(Bool(data=close_gripper))

    ## Utils ##

    def _lookup(self, frame: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        try:
            tf = self._tf_buffer.lookup_transform(self._g("env_frame"), frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), quat_to_matrix(q.x, q.y, q.z, q.w)

    def _camera_pos(self, p_g: np.ndarray, R_g: np.ndarray) -> np.ndarray:
        return p_g + R_g @ self._p_g_cam

    def _set_state(self, state: State):
        if state == State.SEARCH:
            self._search_origin = None
            self._locked = False
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
    node = MepHandleGrasp()
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
