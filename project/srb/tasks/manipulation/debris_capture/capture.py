"""Magnet-style capture of the MEP (debris) by a gripper-less manipulator.

The arm carries a visual-only "capture cylinder" on its last link. Its centre is
the robot-side capture point; the centre of the MEP's marker cylinder
(`gripper_fixture/Cylinder_01`) is the target point. The two cylinders are
deliberately independent in size: the MEP cylinder only marks *where* to capture,
while the arm cylinder visualises the region inside which capture happens.

Once the two points are within `distance_threshold`, a `UsdPhysics.FixedJoint` is
created between the arm link and the MEP body, holding their relative pose at that
instant. Removing the joint (`release()`) returns the MEP to free flight.
"""

from enum import IntEnum
from typing import TYPE_CHECKING, Sequence, Tuple

import torch
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    quat_apply,
    subtract_frame_transforms,
)

if TYPE_CHECKING:
    from srb.core.asset import Articulation, RigidObject

Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]


class CaptureState(IntEnum):
    IDLE = 0
    APPROACH = 1
    CAPTURED = 2


@configclass
class CaptureCfg:
    enable: bool = True

    ## Robot side
    # Link the capture cylinder is rigidly mounted on (relative to the robot prim);
    # None selects the robot's flange link (`canadarm3_large_7` for the Canadarm3)
    robot_link: str | None = None
    # Size of the capture cylinder. It represents the capture region, so it may be
    # (and by default is) much larger than the MEP marker cylinder. Kept equal to
    # `distance_threshold` so the visual region matches the actual capture radius.
    radius: float = 0.6  # CAPTURE_RADIUS [m]
    ## NOTE: Kept short enough that the Canadarm3 wrist camera (0.9 m below the
    ## link origin, i.e. 0.46 m past the flange) stays outside the cylinder.
    length: float = 0.4  # CAPTURE_LENGTH [m]
    color_idle: Tuple[float, float, float] = (0.2, 0.4, 1.0)
    color_approach: Tuple[float, float, float] = (1.0, 0.8, 0.0)
    color_captured: Tuple[float, float, float] = (0.0, 1.0, 0.2)
    opacity: float = 0.35

    ## MEP side
    # Marker cylinder whose centre is the capture target (relative to the MEP prim)
    marker_relpath: str = "gripper_fixture/Cylinder_01"

    ## Detection
    ## NOTE: Doubled from the original 0.3 m so the arm no longer has to line up
    ## exactly on the MEP marker centre -- getting close to it is enough. Kept well
    ## under `approach_distance` so a distant arm still cannot capture by accident.
    distance_threshold: float = 0.6  # CAPTURE_DISTANCE_THRESHOLD [m]
    approach_distance: float = 2.0  # IDLE -> APPROACH below this distance [m]
    # After a release, the MEP must first get this multiple of the threshold away
    # before it can be captured again (otherwise it would be re-captured instantly)
    rearm_factor: float = 1.5
    # Minimum change of distance between two "[CAPTURE] Distance" log lines [m]
    log_distance_step: float = 0.05


def capture_cylinder_pose(cfg: CaptureCfg, flange_offset: Pose) -> Pose:
    """Pose of the capture cylinder centre in the link frame.

    The cylinder starts at the flange face and extends `length` along the flange's
    +Z axis, i.e. away from the arm, where the gripper used to be.
    """
    pos, rot = combine_frame_transforms(
        torch.tensor([flange_offset[0]]),
        torch.tensor([flange_offset[1]]),
        torch.tensor([[0.0, 0.0, 0.5 * cfg.length]]),
    )
    return tuple(pos[0].tolist()), tuple(rot[0].tolist())  # type: ignore


def spawn_capture_cylinder(cfg: CaptureCfg, link_prim_path: str, flange_offset: Pose):
    """Spawn the visual-only capture cylinder under the arm link (env regex allowed).

    It has no collision, rigid body or mass on purpose: it only visualises the
    capture region and follows the link because it is parented to it.
    """
    from srb.core.sim import CylinderCfg, PreviewSurfaceCfg

    pos, rot = capture_cylinder_pose(cfg, flange_offset)
    shape_cfg = CylinderCfg(
        radius=cfg.radius,
        height=cfg.length,
        axis="Z",
        visual_material=PreviewSurfaceCfg(
            diffuse_color=cfg.color_idle, opacity=cfg.opacity
        ),
    )
    shape_cfg.func(
        f"{link_prim_path}/capture_cylinder",
        shape_cfg,
        translation=pos,
        orientation=rot,
    )


class CaptureManager:
    """Distance-triggered capture state machine with a physics fixed-joint attachment."""

    def __init__(
        self,
        cfg: CaptureCfg,
        *,
        stage: Usd.Stage,
        env_prim_paths: Sequence[str],
        robot: "Articulation",
        mep: "RigidObject",
        flange_offset: Pose,
    ):
        self.cfg = cfg
        self._stage = stage
        self._robot = robot
        self._mep = mep
        num_envs = len(env_prim_paths)
        device = robot.device

        robot_name = robot.cfg.prim_path.rsplit("/", 1)[-1]
        mep_name = mep.cfg.prim_path.rsplit("/", 1)[-1]
        self._link_prim_paths = [
            f"{env}/{robot_name}/{cfg.robot_link}" for env in env_prim_paths
        ]
        self._mep_prim_paths = [f"{env}/{mep_name}" for env in env_prim_paths]
        self._joint_prim_paths = [f"{env}/capture_joint" for env in env_prim_paths]

        body_ids, body_names = robot.find_bodies(cfg.robot_link)
        if len(body_ids) != 1:
            raise ValueError(
                f"Capture link '{cfg.robot_link}' must match exactly one robot body, "
                f"got {body_names}"
            )
        self._link_body_id: int = body_ids[0]

        # Robot capture point: centre of the capture cylinder, in the link frame
        self._capture_point_link = torch.tensor(
            [capture_cylinder_pose(cfg, flange_offset)[0]], device=device
        ).repeat(num_envs, 1)
        # MEP capture point: centre of the marker cylinder, in the MEP body frame
        self._capture_point_mep = torch.tensor(
            [self._marker_offset_in_body(path) for path in self._mep_prim_paths],
            device=device,
        )
        # Face normals of the two capture surfaces, used only to report how square the
        # attachment came out (see `_attach`). The marker is a puck whose axis is the
        # MEP's own long axis, so a tilt here is a tilt of the peg -- which is why the
        # vision servo aligns to it before docking.
        self._marker_axis_mep = torch.tensor(
            [self._marker_axis_in_body(path) for path in self._mep_prim_paths],
            device=device,
        )
        self._capture_axis_link = torch.tensor(
            [quat_apply(
                torch.tensor([capture_cylinder_pose(cfg, flange_offset)[1]]),
                torch.tensor([[0.0, 0.0, 1.0]]),
            )[0].tolist()],
            device=device,
        ).repeat(num_envs, 1)

        # World-space USD scale of each jointed body (see `_attach`)
        self._link_scales = torch.tensor(
            [_world_scale(self._stage, path) for path in self._link_prim_paths],
            device=device,
        )
        self._mep_scales = torch.tensor(
            [_world_scale(self._stage, path) for path in self._mep_prim_paths],
            device=device,
        )

        self.state = torch.full(
            (num_envs,), int(CaptureState.IDLE), dtype=torch.long, device=device
        )
        self.distance = torch.full((num_envs,), float("inf"), device=device)
        self._armed = torch.ones(num_envs, dtype=torch.bool, device=device)
        self._last_logged_distance = [float("inf")] * num_envs

        mep_mass = mep.root_physx_view.get_masses().sum(dim=-1)
        for env_id in range(num_envs):
            self._log(
                env_id,
                f"Capture cylinder r={cfg.radius:.3f} m, L={cfg.length:.3f} m on "
                f"'{cfg.robot_link}' | threshold {cfg.distance_threshold:.3f} m | "
                f"MEP mass {float(mep_mass[env_id]):.0f} kg",
            )
            self._log(env_id, f"State: {CaptureState.IDLE.name}")

    ## Geometry

    def _marker_offset_in_body(self, mep_prim_path: str) -> Tuple[float, float, float]:
        marker_path = f"{mep_prim_path}/{self.cfg.marker_relpath}"
        marker = self._stage.GetPrimAtPath(marker_path)
        if not marker.IsValid():
            raise ValueError(f"MEP capture marker prim not found: {marker_path}")
        cache = UsdGeom.XformCache()
        root_tf = Gf.Transform(
            cache.GetLocalToWorldTransform(self._stage.GetPrimAtPath(mep_prim_path))
        )
        marker_pos = cache.GetLocalToWorldTransform(marker).ExtractTranslation()
        # The MEP prim carries a USD scale but its physics body frame does not, so the
        # world-space offset is expressed in the rotation-only (unscaled) root frame.
        offset = (
            root_tf.GetRotation()
            .GetInverse()
            .TransformDir(marker_pos - root_tf.GetTranslation())
        )
        return (offset[0], offset[1], offset[2])

    def _marker_axis_in_body(self, mep_prim_path: str) -> Tuple[float, float, float]:
        """Unit axis of the marker cylinder, in the MEP body frame.

        A UsdGeom.Cylinder is authored along its own +Z; the marker's orientation puts
        that along the MEP's +Y, i.e. pointing out of the end the arm approaches.
        """
        marker = self._stage.GetPrimAtPath(f"{mep_prim_path}/{self.cfg.marker_relpath}")
        cache = UsdGeom.XformCache()
        root_rot = Gf.Transform(
            cache.GetLocalToWorldTransform(self._stage.GetPrimAtPath(mep_prim_path))
        ).GetRotation()
        marker_rot = Gf.Transform(cache.GetLocalToWorldTransform(marker)).GetRotation()
        axis = root_rot.GetInverse().TransformDir(
            marker_rot.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        )
        axis = axis.GetNormalized()
        return (axis[0], axis[1], axis[2])

    def capture_tilt_deg(self, env_id: int) -> float:
        """Angle between the two capture faces; 0 means they are flush.

        The marker normal points out at the arm and the capture cylinder's axis points
        in at the MEP, so a flush dock has them exactly anti-parallel.
        """
        marker_axis_w = quat_apply(
            self._mep.data.root_quat_w[env_id].unsqueeze(0),
            self._marker_axis_mep[env_id].unsqueeze(0),
        )[0]
        capture_axis_w = quat_apply(
            self._robot.data.body_quat_w[env_id, self._link_body_id].unsqueeze(0),
            self._capture_axis_link[env_id].unsqueeze(0),
        )[0]
        cos = torch.clamp(-torch.dot(marker_axis_w, capture_axis_w), -1.0, 1.0)
        return float(torch.rad2deg(torch.arccos(cos)))

    @property
    def robot_capture_pos_w(self) -> torch.Tensor:
        link_pos = self._robot.data.body_pos_w[:, self._link_body_id]
        link_quat = self._robot.data.body_quat_w[:, self._link_body_id]
        return link_pos + quat_apply(link_quat, self._capture_point_link)

    @property
    def mep_capture_pos_w(self) -> torch.Tensor:
        return self._mep.data.root_pos_w + quat_apply(
            self._mep.data.root_quat_w, self._capture_point_mep
        )

    ## State machine

    def update(self):
        """Evaluate the capture condition. Call once after every physics step."""
        if not self.cfg.enable:
            return

        self.distance = torch.norm(
            self.robot_capture_pos_w - self.mep_capture_pos_w, dim=-1
        )
        rearmed = ~self._armed & (
            self.distance > self.cfg.rearm_factor * self.cfg.distance_threshold
        )
        self._armed |= rearmed

        for env_id in range(self.state.shape[0]):
            if rearmed[env_id]:
                self._log(env_id, "Capture re-armed")
            state = CaptureState(int(self.state[env_id]))
            if state == CaptureState.CAPTURED:
                continue
            distance = float(self.distance[env_id])

            if distance <= self.cfg.distance_threshold and bool(self._armed[env_id]):
                self._log(env_id, f"Distance to MEP: {distance:.3f} m")
                self._log(env_id, "Capture condition satisfied")
                self._attach(env_id)
                continue

            next_state = (
                CaptureState.APPROACH
                if distance <= self.cfg.approach_distance
                else CaptureState.IDLE
            )
            if next_state != state:
                self._set_state(env_id, next_state)
                self._last_logged_distance[env_id] = float("inf")
            if (
                next_state == CaptureState.APPROACH
                and abs(distance - self._last_logged_distance[env_id])
                >= self.cfg.log_distance_step
            ):
                self._log(env_id, f"Distance to MEP: {distance:.3f} m")
                self._last_logged_distance[env_id] = distance

    def release(self, env_ids: Sequence[int] | None = None):
        """Remove the attachment, returning the MEP to free flight."""
        if env_ids is None:
            env_ids = range(self.state.shape[0])
        for env_id in env_ids:
            env_id = int(env_id)
            joint_path = self._joint_prim_paths[env_id]
            if self._stage.GetPrimAtPath(joint_path).IsValid():
                self._stage.RemovePrim(joint_path)
                self._log(env_id, "MEP released from Canadarm3")
                # The MEP is still inside the capture region right after a release
                self._armed[env_id] = False
                self._log(
                    env_id,
                    "Capture disarmed until the MEP is more than "
                    f"{self.cfg.rearm_factor * self.cfg.distance_threshold:.3f} m away",
                )
            if int(self.state[env_id]) != CaptureState.IDLE:
                self._set_state(env_id, CaptureState.IDLE)
            self._last_logged_distance[env_id] = float("inf")

    def status(self, env_id: int = 0) -> str:
        return (
            f"State: {CaptureState(int(self.state[env_id])).name} | "
            f"Distance to MEP: {float(self.distance[env_id]):.3f} m "
            f"(threshold {self.cfg.distance_threshold:.3f} m)"
        )

    def _attach(self, env_id: int):
        # Freeze the relative pose of the two bodies exactly as it is now, so the joint
        # starts with zero violation and nothing snaps. The joint frame is placed at the
        # robot capture point (right next to the MEP marker) rather than at either body
        # origin: the MEP origin is ~8 m away, and any angular compliance of the joint
        # would otherwise be amplified by that lever arm at the point that matters.
        link_quat = self._robot.data.body_quat_w[env_id, self._link_body_id]
        anchor_pos_w = self.robot_capture_pos_w[env_id]
        anchor_in_mep, link_rot_in_mep = subtract_frame_transforms(
            self._mep.data.root_pos_w[env_id].unsqueeze(0),
            self._mep.data.root_quat_w[env_id].unsqueeze(0),
            anchor_pos_w.unsqueeze(0),
            link_quat.unsqueeze(0),
        )
        anchor_in_link = self._capture_point_link[env_id]

        # PhysX applies each body's USD scale to the joint's local positions (the same
        # convention `omni.physx.scripts.utils.createJoint` follows), so the metric
        # offsets computed above are divided by the scale of the body they refer to.
        local_pos0 = anchor_in_link / self._link_scales[env_id]
        local_pos1 = anchor_in_mep[0] / self._mep_scales[env_id]
        local_rot1 = link_rot_in_mep[0].tolist()

        joint = UsdPhysics.FixedJoint.Define(self._stage, self._joint_prim_paths[env_id])
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._link_prim_paths[env_id])])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(self._mep_prim_paths[env_id])])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0.tolist()))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1.tolist()))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(local_rot1[0], *local_rot1[1:]))
        # Leave the arm's reduced-coordinate articulation untouched: the MEP is held by
        # a separate maximal-coordinate constraint instead of becoming a new link.
        joint.CreateExcludeFromArticulationAttr().Set(True)
        ## NOTE: PhysX logs "CreateJoint - found a joint with disjointed body
        ## transforms" for this joint. It checks the bodies' USD transforms, which Isaac
        ## Lab never writes back during simulation (they still hold the spawn poses).
        ## The local frames above come from the live physics poses, so nothing snaps.

        # The fixed joint freezes whatever relative pose the arm arrived with, so this
        # number is the whole story on how square the capture was. `mep_suction_capture`
        # aligns to the marker face before docking to keep it small; report it either
        # way, because a large tilt here is exactly what makes peg-in-hole impossible.
        self._log(
            env_id,
            f"Capture tilt: {self.capture_tilt_deg(env_id):.2f} deg between the "
            "marker face and the capture face (0 = flush)",
        )
        self._log(env_id, "MEP attached to Canadarm3")
        self._set_state(env_id, CaptureState.CAPTURED)

    def _set_state(self, env_id: int, state: CaptureState):
        self.state[env_id] = int(state)
        self._log(env_id, f"State: {state.name}")
        self._set_cylinder_color(
            env_id,
            {
                CaptureState.IDLE: self.cfg.color_idle,
                CaptureState.APPROACH: self.cfg.color_approach,
                CaptureState.CAPTURED: self.cfg.color_captured,
            }[state],
        )

    def _set_cylinder_color(self, env_id: int, color: Tuple[float, float, float]):
        prim = self._stage.GetPrimAtPath(
            f"{self._link_prim_paths[env_id]}/capture_cylinder"
        )
        if not prim.IsValid():
            return
        for child in Usd.PrimRange(prim):
            if child.IsA(UsdShade.Shader):
                UsdShade.Shader(child).CreateInput(
                    "diffuseColor", Sdf.ValueTypeNames.Color3f
                ).Set(Gf.Vec3f(*color))

    def _log(self, env_id: int, msg: str):
        prefix = (
            "[CAPTURE]" if self.state.shape[0] == 1 else f"[CAPTURE][env {env_id}]"
        )
        print(f"{prefix} {msg}", flush=True)


def _world_scale(stage: Usd.Stage, prim_path: str) -> Tuple[float, float, float]:
    scale = Gf.Transform(
        UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))
    ).GetScale()
    return (scale[0], scale[1], scale[2])
