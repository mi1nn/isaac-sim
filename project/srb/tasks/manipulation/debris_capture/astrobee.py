"""Astrobee free-flyer: satellite observation / monitoring camera platform.

The Astrobee flies around the target satellite and keeps it -- and what happens around
it (MRV, Canadarm3, MEP, docking) -- in the view of its camera. That camera feed is its
only output (ROS 2 `sensor_msgs/Image`, `/<astrobee.ros_namespace>/<astrobee.image_topic>`).

What it does NOT do: no MEP search / capture, no AprilTag or marker detection, no pose
estimation or vision tracking, no satellite state / angular-velocity estimate, no docking
feasibility (no DOCKING_AVAILABLE / DOCKING_UNAVAILABLE), no mission decision, no command
to the Canadarm3 / MEP / docking pipeline. Nothing it does feeds back into the mission.

Motion: a simplified, kinematic 6-DoF flight (no propulsion, drag or gravity model; the
model is visual only -- no rigid body, no collider, so it cannot touch the scene). The
path is anchored at the satellite's *simulation* pose (ground truth, only to place the
path; it is not measured, estimated or published):

    ASTROBEE_IDLE (start_delay_s) -> ASTROBEE_APPROACH (straight in to point 1)
    -> ASTROBEE_OBSERVATION_START -> ASTROBEE_OBSERVING (dwell at each inspection point,
    smooth arc to the next one, `loops` times; 0 = until the run ends)
    -> ASTROBEE_OBSERVATION_COMPLETE (holds at the last point, camera still on)

The states are internal (console log only): they are not published and not shown in the UI.

Frames: world W (Z up, metres). Astrobee body frame B as in the NASA description: +X
forward, +Y starboard, +Z down, origin at the body centre. The camera sits at the SciCam
mount of the NASA geometry config (`sci_cam_transform`, B frame, scaled with the model)
and looks along +X_B. Quaternions are (w, x, y, z).

The planner below is pure numpy (`project/tests/test_astrobee_observer.py`); the Isaac Sim
side (`AstrobeeObserver`) imports Isaac Lab / rclpy lazily.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .frames import Frame

ASTROBEE_STATES = (
    "ASTROBEE_IDLE",
    "ASTROBEE_APPROACH",
    "ASTROBEE_OBSERVATION_START",
    "ASTROBEE_OBSERVING",
    "ASTROBEE_OBSERVATION_COMPLETE",
)

# Planner phases (the observer turns the first "observe" step into ASTROBEE_OBSERVATION_START)
_PHASE_STATE = {
    "idle": "ASTROBEE_IDLE",
    "approach": "ASTROBEE_APPROACH",
    "observe": "ASTROBEE_OBSERVING",
    "complete": "ASTROBEE_OBSERVATION_COMPLETE",
}

# Camera optical axis in B: forward +X_B, image up = -Z_B (B is Z-down). Isaac Lab
# "world" camera convention is forward +X, up +Z, so the camera frame in B is Rx(pi).
CAM_IN_BODY_QUAT = (0.0, 1.0, 0.0, 0.0)

##############
### Config ###
##############


@dataclass
class AstrobeeCameraCfg:
    """RGB observation camera on the Astrobee (monitoring feed only, never processed)."""

    name: str = "cam_astrobee"
    width: int = 640
    height: int = 480
    horizontal_fov_deg: float = 70.0
    horizontal_aperture_mm: float = 20.955
    clipping_range_m: List[float] = field(default_factory=lambda: [0.1, 1000.0])
    # NASA `sci_cam_transform` translation (B frame, unscaled model metres)
    mount_pos_body_m: List[float] = field(default_factory=lambda: [0.118, 0.0, -0.096])
    # ROS 2 image rate [Hz] (simulation time)
    publish_rate_hz: float = 5.0
    # Save a PNG frame every this many seconds (0: off) under logs/<tag>_astrobee/
    save_every_s: float = 0.0

    @property
    def focal_length_mm(self) -> float:
        return self.horizontal_aperture_mm / (2.0 * math.tan(math.radians(self.horizontal_fov_deg) / 2.0))


@dataclass
class AstrobeeCfg:
    """`astrobee:` section of `vision_capture.yaml`."""

    enabled: bool = True
    # Under `assets/space_asset/` (built by `project/scripts/build_astrobee_usd.py`)
    usd_relpath: str = "astrobee/astrobee.usd"
    prim_name: str = "astrobee"
    # Display scale of the 0.32 m NASA model, like the other scaled assets of this scene
    # (satellite x3.5, MRV hull x3.4); the camera mount scales with it
    scale: float = 3.0
    # Path around the satellite. The horizontal ring radius is the satellite's
    # horizontal AABB half-diagonal + `orbit_margin_m`; the ring sits `elevation_deg`
    # above the satellite centre as seen from it
    start_delay_s: float = 0.0
    approach_distance_m: float = 20.0
    approach_speed_mps: float = 1.0
    orbit_margin_m: float = 25.0
    elevation_deg: float = 25.0
    # Inspection points, azimuth [deg] about world +Z measured from the horizontal
    # direction satellite centre -> docking port (0 = straight in front of the port,
    # where the MRV / MEP come in). Visited in list order, always turning the same way.
    inspection_azimuths_deg: List[float] = field(default_factory=lambda: [45.0, 135.0, 225.0, 315.0])
    dwell_s: float = 5.0
    transit_speed_mps: float = 2.0
    # Camera aim: 0 = satellite centre, 1 = docking port; in between keeps the satellite
    # and the docking area (MEP / MRV side) in the same view
    look_at_dock_weight: float = 0.5
    # Full loops over the inspection points before ASTROBEE_OBSERVATION_COMPLETE (0: forever)
    loops: int = 0
    # ROS 2 (only when `ros.enabled`): camera image only
    ros_namespace: str = "astrobee"
    ros_node_name: str = "astrobee_observer"
    image_topic: str = "camera/image_raw"
    camera: AstrobeeCameraCfg = field(default_factory=AstrobeeCameraCfg)


def validate_astrobee_cfg(cfg: AstrobeeCfg):
    c = cfg.camera
    if cfg.scale <= 0.0:
        raise ValueError("astrobee.scale must be > 0")
    if cfg.approach_speed_mps <= 0.0 or cfg.transit_speed_mps <= 0.0:
        raise ValueError("astrobee.approach_speed_mps / transit_speed_mps must be > 0")
    if cfg.approach_distance_m < 0.0 or cfg.orbit_margin_m < 0.0 or cfg.dwell_s < 0.0 or cfg.start_delay_s < 0.0:
        raise ValueError("astrobee distances / times must be >= 0")
    if not 0.0 <= cfg.look_at_dock_weight <= 1.0:
        raise ValueError("astrobee.look_at_dock_weight must be in [0, 1]")
    if not -80.0 <= cfg.elevation_deg <= 80.0:
        raise ValueError("astrobee.elevation_deg must be in [-80, 80]")
    if len(cfg.inspection_azimuths_deg) < 1:
        raise ValueError("astrobee.inspection_azimuths_deg needs at least one point")
    if int(cfg.loops) < 0:
        raise ValueError("astrobee.loops must be >= 0 (0 = forever)")
    if not cfg.ros_namespace.strip("/") or not cfg.image_topic.strip("/"):
        raise ValueError("astrobee.ros_namespace / image_topic must not be empty")
    cfg.ros_namespace = cfg.ros_namespace.strip("/")
    cfg.image_topic = cfg.image_topic.strip("/")
    if c.width < 16 or c.height < 16:
        raise ValueError("astrobee.camera.width / .height must be >= 16 px")
    if not 1.0 < c.horizontal_fov_deg < 179.0:
        raise ValueError("astrobee.camera.horizontal_fov_deg must be in (1, 179)")
    lo, hi = c.clipping_range_m
    if not 0.0 < lo < hi:
        raise ValueError("astrobee.camera.clipping_range_m must be [near, far] with 0 < near < far")
    if c.publish_rate_hz <= 0.0 or c.save_every_s < 0.0:
        raise ValueError("astrobee.camera.publish_rate_hz must be > 0 and save_every_s >= 0")
    if len(c.mount_pos_body_m) != 3:
        raise ValueError("astrobee.camera.mount_pos_body_m must be [x, y, z]")


###############
### Planner ###
###############


def smoothstep(x: float) -> float:
    """C1 ease-in / ease-out on [0, 1] (zero velocity at both ends)."""
    x = min(1.0, max(0.0, float(x)))
    return x * x * (3.0 - 2.0 * x)


def look_at_rotation(eye, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Rotation of the Astrobee body B (columns = axes in W): +X_B towards `target`,
    +Z_B (down) as close as possible to -`up`, +Y_B = Z_B x X_B (right-handed)."""
    x = np.asarray(target, dtype=float) - np.asarray(eye, dtype=float)
    n = np.linalg.norm(x)
    if n < 1e-9:
        return np.eye(3)
    x /= n
    down = -np.asarray(up, dtype=float)
    z = down - float(down @ x) * x
    if np.linalg.norm(z) < 1e-6:  # looking straight up / down: any horizontal "down"
        z = np.array([1.0, 0.0, 0.0]) - x[0] * x
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


class ObservationPath:
    """Offsets from the satellite centre (world axes) as a function of time since start.

    Pure function of time, so the flight is smooth and repeatable; the satellite's own
    translation / rotation is added by the caller (`center_w`), never predicted here.
    """

    def __init__(self, cfg: AstrobeeCfg, ring_radius_m: float, ref_dir_w):
        self.cfg = cfg
        self.r = float(ring_radius_m)
        u = np.asarray(ref_dir_w, dtype=float).copy()
        u[2] = 0.0
        if np.linalg.norm(u) < 1e-9:
            u = np.array([1.0, 0.0, 0.0])
        self.u = u / np.linalg.norm(u)
        self.v = np.cross([0.0, 0.0, 1.0], self.u)
        self.h = self.r * math.tan(math.radians(cfg.elevation_deg))
        az = [math.radians(a) for a in cfg.inspection_azimuths_deg]
        self.az = az
        # Signed steps between consecutive points, always turning the same way (+)
        self.steps = [((az[(i + 1) % len(az)] - az[i]) % (2.0 * math.pi)) or (2.0 * math.pi if len(az) == 1 else 0.0)
                      for i in range(len(az))]
        self.transit_s = [self.r * s / cfg.transit_speed_mps for s in self.steps]
        self.leg_s = [cfg.dwell_s + t for t in self.transit_s]
        self.loop_s = sum(self.leg_s)
        self.approach_s = cfg.approach_distance_m / cfg.approach_speed_mps
        self.observe_t0 = cfg.start_delay_s + self.approach_s

    def ring_point(self, azimuth_rad: float) -> np.ndarray:
        return self.r * (math.cos(azimuth_rad) * self.u + math.sin(azimuth_rad) * self.v) + np.array([0.0, 0.0, self.h])

    def inspection_points(self) -> List[np.ndarray]:
        return [self.ring_point(a) for a in self.az]

    def approach_start(self) -> np.ndarray:
        p0 = self.ring_point(self.az[0])
        return p0 + self.cfg.approach_distance_m * p0 / np.linalg.norm(p0)

    def sample(self, t: float) -> Tuple[str, np.ndarray, int]:
        """(phase, offset from the satellite centre [m, world axes], inspection point index)."""
        c = self.cfg
        if t < c.start_delay_s:
            return "idle", self.approach_start(), 0
        if t < self.observe_t0 and self.approach_s > 0.0:
            s = smoothstep((t - c.start_delay_s) / self.approach_s)
            p0 = self.ring_point(self.az[0])
            return "approach", (1.0 - s) * self.approach_start() + s * p0, 0
        tau = t - self.observe_t0
        n = len(self.az)
        if self.loop_s <= 0.0:
            return ("complete" if c.loops > 0 else "observe"), self.ring_point(self.az[0]), 0
        loop = int(tau // self.loop_s)
        if c.loops > 0 and loop >= c.loops:
            return "complete", self.ring_point(self.az[0]), 0
        tau -= loop * self.loop_s
        for i in range(n):
            if tau < c.dwell_s:
                return "observe", self.ring_point(self.az[i]), i
            tau -= c.dwell_s
            if tau < self.transit_s[i]:
                a = self.az[i] + self.steps[i] * smoothstep(tau / self.transit_s[i])
                return "observe", self.ring_point(a), i
            tau -= self.transit_s[i]
        return "observe", self.ring_point(self.az[0]), 0


def ring_radius_from_aabb(aabb_min, aabb_max, margin_m: float) -> float:
    """Horizontal half-diagonal of a world AABB + margin [m]."""
    size = np.asarray(aabb_max, dtype=float) - np.asarray(aabb_min, dtype=float)
    return float(0.5 * math.hypot(size[0], size[1]) + margin_m)


def camera_world_pose(body: Frame, cfg: AstrobeeCfg) -> Frame:
    """Camera frame in W (Isaac Lab "world" convention: +X forward, +Z up)."""
    mount = Frame.from_pos_quat(np.asarray(cfg.camera.mount_pos_body_m, dtype=float) * cfg.scale, CAM_IN_BODY_QUAT)
    return body @ mount


################
### Isaac Sim ###
################


class AstrobeeCameraPublisher:
    """ROS 2 publisher of the Astrobee camera image. Nothing else is published."""

    def __init__(self, cfg: AstrobeeCfg, distro: str):
        from .ros_interface import _import_rclpy

        rclpy = self.rclpy = _import_rclpy(distro)
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        self._image = Image
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        from rclpy.node import Node

        self.node = Node(cfg.ros_node_name, namespace=f"/{cfg.ros_namespace}", start_parameter_services=False)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        # Same QoS as the existing camera topics (`cam_wrist/image_raw`)
        self.pub = self.node.create_publisher(Image, cfg.image_topic, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.frame_id = f"{cfg.ros_namespace}/{cfg.camera.name}"
        self.topic = f"/{cfg.ros_namespace}/{cfg.image_topic}"
        print(f"[ASTROBEE] ROS 2 camera feed on {self.topic} (sensor_msgs/Image rgb8)", flush=True)

    def publish(self, t: float, image: np.ndarray):
        from builtin_interfaces.msg import Time

        img = np.ascontiguousarray(image, dtype=np.uint8)
        m = self._image()
        sec = int(math.floor(t))
        m.header.stamp = Time(sec=sec, nanosec=int((t - sec) * 1e9))
        m.header.frame_id = self.frame_id
        m.height, m.width, m.encoding, m.is_bigendian = int(img.shape[0]), int(img.shape[1]), "rgb8", 0
        m.step = int(img.shape[1]) * 3
        m.data = img.tobytes()
        self.pub.publish(m)

    def close(self):
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
            if self._owns_context and self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception as e:  # never let ROS teardown hide the run result
            print(f"[ASTROBEE] ROS shutdown: {e}", flush=True)


class AstrobeeObserver:
    """Flies the Astrobee model + camera along `ObservationPath` and streams the image.

    `step(t)` before each physics step (poses for the next render), `after_render(t)`
    after `scene.update` (image grab / publish at `camera.publish_rate_hz`).
    """

    def __init__(self, task, cfg: AstrobeeCfg, ros_enabled: bool, ros_distro: str, headless: bool,
                 out_dir: Optional[Path] = None, label: str = "run"):
        import torch
        from pxr import Usd, UsdGeom

        from .docking import prim_frame

        self._torch = torch
        self.cfg = cfg
        self.task = task
        self.headless = headless
        self.xform = task.scene.extras[cfg.prim_name]
        self._xform_device = self.xform.get_world_poses()[0].device
        self.camera = task.scene[cfg.camera.name]
        self.sat = task._satellite
        env_path = task.scene.env_prim_paths[0]
        self.camera_path = f"{env_path}/{cfg.camera.name}"
        stage = task.scene.stage

        ## Satellite centre (AABB centre) in the satellite root frame, measured once on the
        ## USD stage; the live position then follows the simulated root pose
        geo = task.docking_geometry
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        box = cache.ComputeWorldBound(stage.GetPrimAtPath(geo.sat_prim_path)).ComputeAlignedRange()
        lo, hi = np.array(box.GetMin(), dtype=float), np.array(box.GetMax(), dtype=float)
        sat_usd = prim_frame(stage, geo.sat_prim_path)
        self.center_in_sat = sat_usd.inv().point(0.5 * (lo + hi))
        radius = ring_radius_from_aabb(lo, hi, cfg.orbit_margin_m)
        self.sat_dock = geo.sat_dock  # satellite root frame (docking.py SAT_DOCK_POINT)
        center_w = self.center_w()
        dock_w = (self.sat_frame() @ geo.sat_dock).pos
        self.path = ObservationPath(cfg, radius, dock_w - center_w)
        self.ros: Optional[AstrobeeCameraPublisher] = None
        if ros_enabled:
            self.ros = AstrobeeCameraPublisher(cfg, ros_distro)
        self.state: Optional[str] = None
        self.frames = 0
        self.saved = 0
        self._next_pub_t = 0.0
        self._next_save_t = 0.0
        self._point = -1
        self._window = None
        # Flight speed for the GUI "Docking monitor" (display only): from consecutive
        # commanded poses, world frame and relative to the satellite centre [m/s]
        self.speed_mps = 0.0
        self.rel_speed_mps = 0.0
        self._last: Optional[Tuple[float, np.ndarray, np.ndarray]] = None
        self.save_dir: Optional[Path] = None
        if cfg.camera.save_every_s > 0.0 and out_dir is not None:
            self.save_dir = Path(out_dir) / f"{label}_astrobee"
            self.save_dir.mkdir(parents=True, exist_ok=True)
            for old in self.save_dir.glob("*.png"):
                old.unlink()
        print(f"[ASTROBEE] observation ring: radius {radius:.1f} m, {self.path.h:.1f} m above the satellite centre "
              f"{np.round(center_w, 2).tolist()} (satellite AABB {np.round(hi - lo, 1).tolist()} m), "
              f"{len(cfg.inspection_azimuths_deg)} inspection points, approach {self.path.approach_s:.0f} s, "
              f"one loop {self.path.loop_s:.0f} s, camera {cfg.camera.width}x{cfg.camera.height} "
              f"FOV {cfg.camera.horizontal_fov_deg:g} deg", flush=True)

    def sat_frame(self) -> Frame:
        d = self.sat.data
        return Frame.from_pos_quat(d.root_pos_w[0].tolist(), d.root_quat_w[0].tolist())

    def center_w(self) -> np.ndarray:
        return self.sat_frame().point(self.center_in_sat)

    def _set_state(self, state: str, detail: str = ""):
        if state != self.state:
            self.state = state
            print(f"[ASTROBEE] {state}{'  ' + detail if detail else ''}", flush=True)

    def step(self, t: float):
        """Place the Astrobee and its camera for time `t` [s, simulation]."""
        torch = self._torch
        phase, offset, idx = self.path.sample(t)
        sat = self.sat_frame()
        center = sat.point(self.center_in_sat)
        pos = center + offset
        aim = center + self.cfg.look_at_dock_weight * ((sat @ self.sat_dock).pos - center)
        body = Frame(pos, look_at_rotation(pos, aim))
        if self._last is not None and t > self._last[0]:
            dt = t - self._last[0]
            self.speed_mps = float(np.linalg.norm(pos - self._last[1])) / dt
            self.rel_speed_mps = float(np.linalg.norm(offset - self._last[2])) / dt
        self._last = (t, pos.copy(), np.asarray(offset, dtype=float).copy())
        if phase == "observe" and self.state in (None, "ASTROBEE_IDLE", "ASTROBEE_APPROACH"):
            self._set_state("ASTROBEE_OBSERVATION_START", "at inspection point 1")
        else:
            self._set_state(_PHASE_STATE[phase])
        if phase == "observe" and idx != self._point:
            self._point = idx
            print(f"[ASTROBEE] inspection point {idx + 1}/{len(self.path.az)} "
                  f"(azimuth {self.cfg.inspection_azimuths_deg[idx]:g} deg)", flush=True)
        dev = self._xform_device
        self.xform.set_world_poses(
            positions=torch.tensor([pos.tolist()], dtype=torch.float32, device=dev),
            orientations=torch.tensor([list(body.quat)], dtype=torch.float32, device=dev),
        )
        cam = camera_world_pose(body, self.cfg)
        cdev = self.camera.device
        self.camera.set_world_poses(
            positions=torch.tensor([cam.pos.tolist()], dtype=torch.float32, device=cdev),
            orientations=torch.tensor([list(cam.quat)], dtype=torch.float32, device=cdev),
            convention="world",
        )

    def after_render(self, t: float):
        """Grab the rendered image and publish it (rate limited)."""
        c = self.cfg.camera
        publish_due = self.ros is not None and t >= self._next_pub_t
        save_due = self.save_dir is not None and t >= self._next_save_t
        if not (publish_due or save_due):
            return
        rgb = self.camera.data.output["rgb"][0].cpu().numpy()[..., :3]
        if publish_due:
            self._next_pub_t = t + 1.0 / c.publish_rate_hz - 1e-9
            self.ros.publish(t, rgb)
            self.frames += 1
        if save_due:
            import cv2

            self._next_save_t = t + c.save_every_s - 1e-9
            cv2.imwrite(str(self.save_dir / f"astrobee_{t:08.2f}s.png"), cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR))
            self.saved += 1

    def open_window(self):
        """GUI: live viewport window of the Astrobee camera."""
        if self.headless:
            return
        try:
            from omni.kit.viewport.utility import create_viewport_window
            from pxr import Sdf

            self._window = create_viewport_window(
                self.cfg.camera.name, width=640, height=480, position_x=700, position_y=60,
                camera_path=Sdf.Path(self.camera_path),
            )
            print(f"[ASTROBEE] opened the '{self.cfg.camera.name}' viewport window", flush=True)
        except Exception as e:  # never let a GUI convenience stop the run
            print(f"[ASTROBEE] could not open the camera window: {e}", flush=True)

    def complete(self):
        """End of the run: the observation stops with it."""
        self._set_state("ASTROBEE_OBSERVATION_COMPLETE")

    def close(self):
        print(f"[ASTROBEE] {self.frames} camera frames published"
              f"{f', {self.saved} saved to {self.save_dir}' if self.save_dir is not None else ''}", flush=True)
        if self.ros is not None:
            self.ros.close()
            self.ros = None
