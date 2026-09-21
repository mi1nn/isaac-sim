"""The client satellite's GPS telemetry link, over ROS 2.

The servicer does not get to read the client's true pose. The client broadcasts where it
thinks it is, the way a cooperative target with GPS and a known bus geometry would:

    client node  --  /client/gps/pose        (PoseStamped, satellite body in world)
                 --  /client/thruster_pose   (PoseStamped, docking frame on the thruster)
    servicer     --> subscribes, and gets a pose that is late, noisy and biased

`GpsPublisher` runs inside the simulation and is the *client*; it is the only place that
touches ground truth. `GpsSubscriber` is the *servicer* and is what the docking state
machine is allowed to read. Keeping them apart is the point: swap `GpsPublisher` for a
real spacecraft and the controller does not change.

The corruption is deliberate and is what makes the depth camera earn its place:

- `rate_hz`      telemetry arrives in steps, not continuously
- `latency_s`    what arrives describes where the client *was*
- `pos_noise_m`  white noise, redrawn per message
- `rot_noise_deg` white attitude noise, redrawn per message
- `bias_pos_m`   a slowly wandering offset that no amount of averaging removes
- `dropout_prob` messages that never arrive

The bias is the reason the insertion depth cannot come from this link: averaging kills
the white noise but not the bias, and a 5 cm axial bias is a crash into the thruster
back plate. The depth camera measures the one number the bias would ruin.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .frames import Frame, quat_wxyz_to_rotmat


@dataclass
class GpsLinkCfg:
    # ROS 2 topics. `thruster_topic` is what the docking controller consumes.
    gps_topic: str = "/client/gps/pose"
    thruster_topic: str = "/client/thruster_pose"
    frame_id: str = "world"
    # Publish rate of the client's telemetry [Hz]
    rate_hz: float = 5.0
    # Age of the pose when it is received [s]
    latency_s: float = 0.2
    # Per-message white noise
    pos_noise_m: float = 0.03
    rot_noise_deg: float = 1.0
    # Slowly wandering offset: magnitude [m] and correlation time [s]
    bias_pos_m: float = 0.05
    bias_tau_s: float = 60.0
    # Fraction of messages that never arrive
    dropout_prob: float = 0.0
    seed: int = 0
    # Keep this many seconds of history so a delayed message can be served
    history_s: float = 5.0


@dataclass
class GpsSample:
    """One received telemetry message, as the servicer sees it."""

    stamp: float  # when the pose was valid (the client's clock)
    received: float  # when the servicer got it
    pose: Frame

    @property
    def age(self) -> float:
        return self.received - self.stamp


class GpsCorruption:
    """Turns a true pose into what the link delivers. Shared by both backends."""

    def __init__(self, cfg: GpsLinkCfg):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self._bias = self.rng.normal(0.0, cfg.bias_pos_m, 3) if cfg.bias_pos_m > 0 else np.zeros(3)

    def step_bias(self, dt: float):
        """Ornstein-Uhlenbeck: the bias wanders but stays bounded."""
        c = self.cfg
        if c.bias_pos_m <= 0.0 or c.bias_tau_s <= 0.0:
            return
        a = math.exp(-dt / c.bias_tau_s)
        self._bias = a * self._bias + math.sqrt(max(0.0, 1.0 - a * a)) * self.rng.normal(0.0, c.bias_pos_m, 3)

    @property
    def bias(self) -> np.ndarray:
        return self._bias.copy()

    def corrupt(self, pose: Frame) -> Frame:
        c = self.cfg
        pos = pose.pos + self._bias
        if c.pos_noise_m > 0.0:
            pos = pos + self.rng.normal(0.0, c.pos_noise_m, 3)
        rot = pose.rot
        if c.rot_noise_deg > 0.0:
            axis = self.rng.normal(0.0, 1.0, 3)
            axis /= max(1e-12, float(np.linalg.norm(axis)))
            angle = math.radians(float(self.rng.normal(0.0, c.rot_noise_deg)))
            rot = _rotmat_axis_angle(axis, angle) @ rot
        return Frame(pos, rot)

    def drops(self) -> bool:
        return self.cfg.dropout_prob > 0.0 and bool(self.rng.random() < self.cfg.dropout_prob)


def _rotmat_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


class GpsLink:
    """Client publisher + servicer subscriber over ROS 2.

    `publish(t, sat_pose, thruster_pose)` is the client side and is called from the
    simulation loop with ground truth. `latest(t)` is the servicer side and returns the
    most recent message that has actually arrived by time `t`, or None.

    The ROS 2 bridge is brought up through `srb.utils.ros.require_ros2_bridge()`, the
    same entry point `srb agent ros` uses, so this works from a standalone script.
    """

    def __init__(self, cfg: GpsLinkCfg, node_name: str = "srb_client_gps"):
        self.cfg = cfg
        self.corruption = GpsCorruption(cfg)
        self._pending: List[GpsSample] = []
        self._delivered: Optional[GpsSample] = None
        self._history: List[Tuple[float, Frame]] = []
        self._next_publish = 0.0
        self._last_t = 0.0
        self.published = 0
        self.dropped = 0

        from srb.utils.ros import require_ros2_bridge

        require_ros2_bridge()
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self._rclpy = rclpy
        self._PoseStamped = PoseStamped
        if not rclpy.ok():
            rclpy.init()
        self._node: Node = rclpy.create_node(node_name)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._pub_gps = self._node.create_publisher(PoseStamped, cfg.gps_topic, qos)
        self._pub_thruster = self._node.create_publisher(PoseStamped, cfg.thruster_topic, qos)
        # The servicer subscribes to its own process here, but the topic is a real one:
        # `ros2 topic echo` sees it, and an external node can publish it instead.
        self._sub = self._node.create_subscription(PoseStamped, cfg.thruster_topic, self._on_thruster, qos)
        self._rx: List[GpsSample] = []
        print(
            f"[GPS] ROS 2 node '{node_name}': publishing {cfg.gps_topic} and {cfg.thruster_topic} "
            f"at {cfg.rate_hz:.1f} Hz (latency {cfg.latency_s*1000:.0f} ms, noise "
            f"{cfg.pos_noise_m*1000:.0f} mm / {cfg.rot_noise_deg:.1f} deg, bias {cfg.bias_pos_m*1000:.0f} mm)",
            flush=True,
        )

    ## Client side -----------------------------------------------------------------

    def publish(self, t: float, sat_pose: Frame, thruster_pose: Frame):
        """Called every control step with ground truth; emits at `rate_hz`."""
        self.corruption.step_bias(max(0.0, t - self._last_t))
        self._last_t = t
        self._history.append((t, thruster_pose))
        cutoff = t - self.cfg.history_s
        self._history = [(ts, p) for ts, p in self._history if ts >= cutoff]
        if t < self._next_publish:
            return
        self._next_publish = t + 1.0 / max(1e-6, self.cfg.rate_hz)
        if self.corruption.drops():
            self.dropped += 1
            return
        # The client stamps the pose it had `latency_s` ago and it lands now
        stamp = t - self.cfg.latency_s
        true_at_stamp = self._pose_at(stamp, thruster_pose)
        self._pub_gps.publish(self._to_msg(stamp, self.corruption.corrupt(sat_pose)))
        self._pub_thruster.publish(self._to_msg(stamp, self.corruption.corrupt(true_at_stamp)))
        self.published += 1

    def _pose_at(self, stamp: float, fallback: Frame) -> Frame:
        for ts, pose in reversed(self._history):
            if ts <= stamp:
                return pose
        return self._history[0][1] if self._history else fallback

    def _to_msg(self, stamp: float, pose: Frame):
        msg = self._PoseStamped()
        msg.header.frame_id = self.cfg.frame_id
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9)) % 1_000_000_000
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pose.pos)
        w, x, y, z = pose.quat
        msg.pose.orientation.w = float(w)
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)
        return msg

    ## Servicer side ---------------------------------------------------------------

    def _on_thruster(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = Frame(
            np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
            quat_wxyz_to_rotmat(
                (msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
            ),
        )
        self._rx.append(GpsSample(stamp=stamp, received=self._last_t, pose=pose))

    def spin(self, t: float):
        """Pump the ROS callbacks; call once per control step before `latest()`."""
        self._last_t = t
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        while True:
            before = len(self._rx)
            self._rclpy.spin_once(self._node, timeout_sec=0.0)
            if len(self._rx) == before:
                break
        if self._rx:
            self._delivered = self._rx[-1]
            self._rx.clear()

    def latest(self, t: float) -> Optional[GpsSample]:
        return self._delivered

    def close(self):
        try:
            self._node.destroy_node()
        except Exception:
            pass


class LoopbackGpsLink(GpsLink):
    """Same contract without ROS, for unit tests and for running with no bridge."""

    def __init__(self, cfg: GpsLinkCfg, node_name: str = "loopback"):
        self.cfg = cfg
        self.corruption = GpsCorruption(cfg)
        self._history: List[Tuple[float, Frame]] = []
        self._inflight: List[GpsSample] = []
        self._delivered: Optional[GpsSample] = None
        self._next_publish = 0.0
        self._last_t = 0.0
        self.published = 0
        self.dropped = 0

    def publish(self, t: float, sat_pose: Frame, thruster_pose: Frame):
        self.corruption.step_bias(max(0.0, t - self._last_t))
        self._last_t = t
        self._history.append((t, thruster_pose))
        cutoff = t - self.cfg.history_s
        self._history = [(ts, p) for ts, p in self._history if ts >= cutoff]
        if t < self._next_publish:
            return
        self._next_publish = t + 1.0 / max(1e-6, self.cfg.rate_hz)
        if self.corruption.drops():
            self.dropped += 1
            return
        stamp = t
        self._inflight.append(
            GpsSample(stamp=stamp, received=t + self.cfg.latency_s, pose=self.corruption.corrupt(thruster_pose))
        )
        self.published += 1

    def spin(self, t: float):
        self._last_t = t
        arrived = [s for s in self._inflight if s.received <= t]
        if arrived:
            self._delivered = arrived[-1]
        self._inflight = [s for s in self._inflight if s.received > t]

    def close(self):
        pass


def make_gps_link(cfg: GpsLinkCfg, use_ros: bool = True, node_name: str = "srb_client_gps") -> GpsLink:
    """`GpsLink` over ROS 2, falling back to the loopback link if the bridge is absent."""
    if not use_ros:
        return LoopbackGpsLink(cfg, node_name)
    try:
        return GpsLink(cfg, node_name)
    except Exception as e:  # no bridge, no rclpy, no daemon
        print(f"[GPS] ROS 2 unavailable ({type(e).__name__}: {e}); using the in-process loopback link", flush=True)
        return LoopbackGpsLink(cfg, node_name)
