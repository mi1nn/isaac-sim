"""Optional ROS 2 interface of the MRV vision capture demo (rclpy, no OmniGraph).

Telemetry out (all topics under /<namespace>/, world frame `ros.world_frame`):

    cam_wrist/image_raw          sensor_msgs/Image        rgb8, the image the vision system uses
    cam_wrist/camera_info        sensor_msgs/CameraInfo   intrinsics of that image (no distortion)
    estimate/cylinder_pose       geometry_msgs/PoseStamped  filtered Cylinder_01 (vision only)
    predicted/cylinder_pose      geometry_msgs/PoseStamped  Cylinder_01 at t + prediction.horizon_sec
    estimate/mep_twist           geometry_msgs/TwistStamped estimated MEP twist (world)
    ee/pose, ee/target_pose      geometry_msgs/PoseStamped  EE contact frame / its target
    gt/cylinder_pose, gt/mep_twist                          ground truth, evaluation only
    state                        std_msgs/String          state machine state (latched)
    captured                     std_msgs/Bool            FixedJoint attached (latched)
    status                       std_msgs/String          JSON: capture metrics, tags, standoff, ...
    docked                       std_msgs/Bool            MEP <-> Client docking FixedJoint (latched, docking phase only)
    client/pose                  geometry_msgs/PoseStamped  Client reference point (SAT_DOCK_POINT) (docking phase)
    orbit/client_error           std_msgs/Float64         distance Client point -> reference orbit [m] (orbit scenario)
    /tf                          tf2_msgs/TFMessage       world -> <ns>/{cylinder_est, cylinder_pred, ee, cam_wrist, cylinder_gt}

Commands in:

    cmd/start           std_msgs/Empty  release the arm (only with ros.require_start_cmd)
    cmd/abort           std_msgs/Empty  stop: hold the joints, final state ABORTED
    cmd/capture_enable  std_msgs/Bool   false: keep tracking but never attach (default true)

Poses are published in simulation time; quaternions are converted from the internal
(w, x, y, z) to ROS (x, y, z, w). The controller never reads the `gt/` topics.
"""

import json
import math
import os
from typing import Dict, Optional

import numpy as np

from .frames import Frame
from .vision import RosVisionCfg


def _import_rclpy(distro: str):
    """rclpy of the Isaac Sim ROS 2 bridge unless a ROS 2 install is already sourced."""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        from srb.utils.ros import enable_ros2_bridge

        if not enable_ros2_bridge(distro=distro):
            raise RuntimeError("ROS 2 (rclpy) is not available: the Isaac Sim 'isaacsim.ros2.bridge' extension did not load")
    import rclpy

    return rclpy


def _finite(x):
    """JSON-safe number (NaN / inf -> None)."""
    return float(x) if isinstance(x, (int, float, np.floating)) and math.isfinite(x) else None


class VisionRosInterface:
    def __init__(self, cfg: RosVisionCfg, image_size, k: np.ndarray, camera_frame: str = "cam_wrist"):
        self.cfg = cfg
        self.ns = cfg.namespace
        self.width, self.height = int(image_size[0]), int(image_size[1])
        self.k = np.asarray(k, dtype=float)
        rclpy = self.rclpy = _import_rclpy(cfg.distro)
        from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Bool, Empty, Float64, String
        from tf2_msgs.msg import TFMessage

        self._msgs = dict(PoseStamped=PoseStamped, TwistStamped=TwistStamped, TransformStamped=TransformStamped,
                          Image=Image, CameraInfo=CameraInfo, String=String, Bool=Bool, Float64=Float64, TFMessage=TFMessage)
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        from rclpy.node import Node

        self.node = Node(cfg.node_name, namespace=f"/{self.ns}", start_parameter_services=False)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        create = self.node.create_publisher
        self.pub: Dict[str, object] = {
            "image": create(Image, "cam_wrist/image_raw", sensor),
            "info": create(CameraInfo, "cam_wrist/camera_info", sensor),
            "est": create(PoseStamped, "estimate/cylinder_pose", reliable),
            "pred": create(PoseStamped, "predicted/cylinder_pose", reliable),
            "twist": create(TwistStamped, "estimate/mep_twist", reliable),
            "ee": create(PoseStamped, "ee/pose", reliable),
            "ee_target": create(PoseStamped, "ee/target_pose", reliable),
            "state": create(String, "state", latched),
            "captured": create(Bool, "captured", latched),
            "status": create(String, "status", reliable),
            # docking phase / orbit scenario (published only when the demo has something to say)
            "docked": create(Bool, "docked", latched),
            "client": create(PoseStamped, "client/pose", reliable),
            "orbit_error": create(Float64, "orbit/client_error", reliable),
        }
        if cfg.publish_ground_truth:
            self.pub["gt"] = create(PoseStamped, "gt/cylinder_pose", reliable)
            self.pub["gt_twist"] = create(TwistStamped, "gt/mep_twist", reliable)
        if cfg.publish_tf:
            self.pub["tf"] = self.node.create_publisher(TFMessage, "/tf", QoSProfile(depth=10))

        self._start = False
        self._abort = False
        self._capture_enabled = True
        self.node.create_subscription(Empty, "cmd/start", self._on_start, reliable)
        self.node.create_subscription(Empty, "cmd/abort", self._on_abort, reliable)
        self.node.create_subscription(Bool, "cmd/capture_enable", self._on_capture_enable, reliable)
        self._last_state: Optional[str] = None
        self._last_captured: Optional[bool] = None
        self._last_docked: Optional[bool] = None
        self.camera_frame = f"{self.ns}/{camera_frame}"
        print(f"[ROS] node /{self.ns}/{cfg.node_name} up, topics under /{self.ns}/ "
              f"(start required: {cfg.require_start_cmd}, RMW {os.environ.get('RMW_IMPLEMENTATION', 'default')})", flush=True)

    ## Commands
    def _on_start(self, _):
        if not self._start:
            print("[ROS] cmd/start received", flush=True)
        self._start = True

    def _on_abort(self, _):
        print("[ROS] cmd/abort received", flush=True)
        self._abort = True

    def _on_capture_enable(self, msg):
        if bool(msg.data) != self._capture_enabled:
            print(f"[ROS] cmd/capture_enable = {bool(msg.data)}", flush=True)
        self._capture_enabled = bool(msg.data)

    def poll(self):
        """Deliver pending command callbacks (non-blocking; call every control step)."""
        self._executor.spin_once(timeout_sec=0.0)

    @property
    def start_received(self) -> bool:
        return self._start or not self.cfg.require_start_cmd

    @property
    def abort_requested(self) -> bool:
        return self._abort

    @property
    def capture_enabled(self) -> bool:
        return self._capture_enabled

    ## Message helpers
    def _stamp(self, t: float):
        from builtin_interfaces.msg import Time

        sec = int(math.floor(t))
        return Time(sec=sec, nanosec=int((t - sec) * 1e9))

    def _pose(self, f: Frame, t: float):
        m = self._msgs["PoseStamped"]()
        m.header.stamp = self._stamp(t)
        m.header.frame_id = self.cfg.world_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(x) for x in f.pos)
        w, x, y, z = f.quat
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = float(w), float(x), float(y), float(z)
        return m

    def _twist(self, v, w, t: float):
        m = self._msgs["TwistStamped"]()
        m.header.stamp = self._stamp(t)
        m.header.frame_id = self.cfg.world_frame
        if v is not None:
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(x) for x in v)
        if w is not None:
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = (float(x) for x in w)
        return m

    def _transform(self, name: str, f: Frame, t: float):
        m = self._msgs["TransformStamped"]()
        m.header.stamp = self._stamp(t)
        m.header.frame_id = self.cfg.world_frame
        m.child_frame_id = f"{self.ns}/{name}"
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(x) for x in f.pos)
        w, x, y, z = f.quat
        m.transform.rotation.w, m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z = float(w), float(x), float(y), float(z)
        return m

    ## Telemetry
    def publish_image(self, t: float, image: np.ndarray):
        """The image the vision system processed (uint8 RGB, height x width x 3)."""
        if not self.cfg.publish_image:
            return
        img = np.ascontiguousarray(image, dtype=np.uint8)
        stamp = self._stamp(t)
        m = self._msgs["Image"]()
        m.header.stamp, m.header.frame_id = stamp, self.camera_frame
        m.height, m.width, m.encoding, m.is_bigendian = int(img.shape[0]), int(img.shape[1]), "rgb8", 0
        m.step = int(img.shape[1]) * 3
        m.data = img.tobytes()
        self.pub["image"].publish(m)
        info = self._msgs["CameraInfo"]()
        info.header.stamp, info.header.frame_id = stamp, self.camera_frame
        info.width, info.height = self.width, self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(x) for x in self.k.reshape(9)]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.k[0, 0], 0.0, self.k[0, 2], 0.0, 0.0, self.k[1, 1], self.k[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub["info"].publish(info)

    def publish_state(self, t: float, state: str, captured: bool, status: dict):
        """State machine (latched, on change) and the JSON status."""
        if state != self._last_state:
            self._last_state = state
            self.pub["state"].publish(self._msgs["String"](data=state))
        if captured != self._last_captured:
            self._last_captured = captured
            self.pub["captured"].publish(self._msgs["Bool"](data=captured))
        payload = {"sim_time_s": t, "state": state, "captured": captured}
        payload.update({k: (_finite(v) if isinstance(v, (int, float, np.floating)) and not isinstance(v, bool) else v) for k, v in status.items()})
        self.pub["status"].publish(self._msgs["String"](data=json.dumps(payload)))

    def publish_docking(self, t: float, docked: bool, client: Frame, orbit_error_m: Optional[float] = None):
        """Docking phase: `docked` (latched, on change), the Client point and, with the orbit scenario,
        its distance to the reference orbit."""
        if docked != self._last_docked:
            self._last_docked = docked
            self.pub["docked"].publish(self._msgs["Bool"](data=bool(docked)))
        self.pub["client"].publish(self._pose(client, t))
        if orbit_error_m is not None:
            self.pub["orbit_error"].publish(self._msgs["Float64"](data=float(orbit_error_m)))

    def publish_poses(self, t: float, est: Optional[Frame], pred: Optional[Frame], ee: Frame, ee_target: Optional[Frame],
                      cam: Optional[Frame], v=None, w=None, gt: Optional[Frame] = None, gt_v=None, gt_w=None):
        p = self.pub
        if est is not None:
            p["est"].publish(self._pose(est, t))
        if pred is not None:
            p["pred"].publish(self._pose(pred, t))
        if v is not None:
            p["twist"].publish(self._twist(v, w, t))
        p["ee"].publish(self._pose(ee, t))
        if ee_target is not None:
            p["ee_target"].publish(self._pose(ee_target, t))
        if self.cfg.publish_ground_truth and gt is not None:
            p["gt"].publish(self._pose(gt, t))
            p["gt_twist"].publish(self._twist(gt_v, gt_w, t))
        if self.cfg.publish_tf:
            tfs = [("ee", ee)]
            if est is not None:
                tfs.append(("cylinder_est", est))
            if pred is not None:
                tfs.append(("cylinder_pred", pred))
            if cam is not None:
                tfs.append(("cam_wrist", cam))
            if self.cfg.publish_ground_truth and gt is not None:
                tfs.append(("cylinder_gt", gt))
            p["tf"].publish(self._msgs["TFMessage"](transforms=[self._transform(n, f, t) for n, f in tfs]))

    def close(self):
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
            if self._owns_context and self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception as e:  # never let ROS teardown hide the run result
            print(f"[ROS] shutdown: {e}", flush=True)
