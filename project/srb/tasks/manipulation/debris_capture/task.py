from dataclasses import MISSING
from typing import Sequence, Tuple

import torch
from pxr import Gf

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import (
    Articulation,
    AssetVariant,
    ExtravehicularScenery,
    MobileRobot,
    Object,
    RigidObject,
    RigidObjectCfg,
)
from srb.core.domain import Domain
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    rotmat_to_rot6d,
    rpy_to_quat,
    scale_transform,
    subtract_frame_transforms,
)
from srb.utils.path import SRB_ASSETS_DIR_SPACE

from .capture import CaptureCfg, CaptureManager, spawn_capture_cylinder

##############
### Config ###
##############


@configclass
class SceneCfg(ManipulationSceneCfg):
    debris: RigidObjectCfg = MISSING  # type: ignore

    ## Custom assets (dynamic rigid bodies)
    ## NOTE: mep.usd bundles 3 sub-objects (Ares1 rocket, Fermi telescope,
    ## gripper_fixture) meant to move/collide together as ONE object. Isaac Lab's
    ## rigid-body resolution otherwise tags each sub-object separately (since the
    ## combined root itself has no rigid-body API), so "mep_combined.usd" is a thin
    ## wrapper that pre-applies a single RigidBodyAPI on the combined root and
    ## CollisionAPI on all 52 of its mesh/shape prims -> the trio spawns and moves
    ## as one rigid body made of many collision shapes.
    ## The trio is used as the "debris" grasp target (see TaskCfg.__post_init__)
    ## instead of the procedural/dataset debris from select_debris(), so it is not
    ## declared as a separate scene entity here.
    satellite: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/satellite",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SPACE.joinpath("satellite_v3.usd").as_posix(),
            # Was 3.5; the docking lengths scale with it (`DockingCfg.reference_scale`)
            scale=(2.4, 2.4, 2.4),
            collision_props=CollisionPropertiesCfg(),
            ## NOTE: The GOES_R meshes author no collision approximation, which PhysX
            ## rejects on a dynamic body (one error per mesh) before silently falling
            ## back to convexHull. Requesting convexHull explicitly keeps the exact same
            ## collision behaviour without the error spam. This is background scenery,
            ## so a hull per mesh is accurate enough and far cheaper than decomposition.
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexHull"
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
        ## NOTE: Pose translated (rotation kept from `no_gripper.usd`) so a peg-in-hole
        ## test between `satellite/GOES_R` and `debris` ("probe" = `Xform_Ares1`, one of
        ## the 3 sub-parts of the combined MEP rigid body) is physically possible.
        ## Original pose was pure background scenery, ~25.9 m from `debris`'s root -- far
        ## outside any possible contact. World-space AABBs (measured at
        ## scale=(3.5,3.5,3.5)) were used to translate `satellite` so its nearest face
        ## sits exactly 1.0 m from the *whole* debris body's AABB (not just Ares1's --
        ## the Fermi telescope sub-part protrudes ~5 m further out and overlapped
        ## satellite at a first, Ares1-only pass, launching debris 3.5 m on the first
        ## step), with Y/Z centred on Ares1 specifically so it still faces the probe:
        ##   debris (whole MEP) AABB: min (-11.142,  4.076, 3.168), max ( 9.420, 14.288, 7.477)
        ##   Ares1 AABB (for facing) : min ( -1.040,  6.692, 3.168), max (  4.308, 14.288, 7.477)
        ##   GOES_R AABB (old)       : min (-24.191, -5.530, 1.969), max ( -0.777, 37.368, 23.423)
        ## The 1.0 m gap is deliberate: spawning with the meshes already interpenetrating
        ## would make PhysX shove them apart violently on the first physics step (the
        ## same "Play explosion" issue documented for the Canadarm3/gripper assembly in
        ## Part 1). Move them the rest of the way together via manual control instead.
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(20.477156, 27.933948, 4.119992),
            rot=(0.707107, 0.707107, 0.0, 0.0),
        ),
    )


@configclass
class EventCfg(ManipulationEventCfg):
    ## NOTE: Ranges are zeroed for manual grasp testing: the debris is placed at
    ## its configured `init_state` pose with zero linear/angular velocity, so it
    ## starts at rest without being made kinematic -- it is still a fully dynamic
    ## rigid body that reacts to gripper contact. Restore the randomization
    ## (see git history for the original ranges) before training.
    randomize_obj_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("debris"),
            "pose_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    ## NOTE: The manipulation base class randomizes the arm joints by +-5 deg on
    ## every reset. Disabled so the arm always starts at the exact joint pose
    ## configured in `Canadarm3.asset_cfg.init_state`.
    randomize_robot_joints: EventTermCfg | None = None


@configclass
class TaskCfg(ManipulationEnvCfg):
    ## Scenario
    domain: Domain = Domain.ORBIT

    ## Assets
    scenery: ExtravehicularScenery | MobileRobot | AssetVariant | None = (
        assets.VenusExpress()
    )
    scenery.asset_cfg.init_state.pos = (-1.65, 0.0, -1.05)
    scenery.asset_cfg.init_state.rot = rpy_to_quat(0.0, 0.0, 90.0)
    scenery.asset_cfg.spawn.scale = (3.4, 3.4, 3.4)
    ## NOTE: The spacecraft is scenery here -- the Canadarm3 base is held by its own
    ## world-fixed root joint, not bolted to this hull, so the hull's collision serves
    ## no purpose in this task. It does cause harm: measured against the configured
    ## initial arm pose, links 0/1/2 clear the (3.4x scaled, yaw 90 deg) hull by
    ## 0.27/0.39/0.47 m, but the long boom (link 3) grazes a solar panel. A *static*
    ## collider touching a dynamic link makes PhysX depenetrate it the moment physics
    ## starts, which is a kick a manual grasp test should not have to fight. Delete
    ## this line to get the hull's collision back.
    scenery.asset_cfg.spawn.collision_props = CollisionPropertiesCfg(
        collision_enabled=False
    )
    pedestal: Object | AssetVariant | None = None

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = True

    ## Capture (magnet-style attachment of the MEP to a gripper-less arm)
    ## NOTE: Only active when the robot has no end-effector (e.g. `env.robot=canadarm3`);
    ## the capture cylinder takes the place of the gripper on the flange.
    capture: CaptureCfg = CaptureCfg()

    ## MEP capture marker, as edited in the reference stage `no_gripper.usd`: the
    ## marker cylinder is moved/shortened and the disc mount next to it is disabled.
    ## Set `debris_marker_xform` to None to keep the asset's own marker placement.
    debris_marker_xform: (
        Tuple[
            Tuple[float, float, float],
            Tuple[float, float, float, float],
            Tuple[float, float, float],
        ]
        | None
    ) = (
        (50.63112171724094, 19.997228065784522, -0.1790183312654655),
        (
            0.7064827155716957,
            -0.7057925476247107,
            0.036989020055530245,
            0.03702519022496403,
        ),
        (0.15000000596046448, 0.14998489618301392, 0.10000000149011612),
    )
    debris_inactive_relpaths: Tuple[str, ...] = ("gripper_fixture/DiscMount",)

    def __post_init__(self):
        super().__post_init__()

        # Keep the skydome (Earth) orientation fixed instead of randomizing it
        self.events.randomize_skydome_orientation = None

        # Scene: Debris
        # Grasp target is the combined 3-asset mep (Ares1 + Fermi telescope +
        # gripper_fixture), spawned close to the manipulator's reach instead of
        # its original hand-placed background pose. Tune `pos`/`scale` below
        # while visually checking placement (e.g. via `srb agent zero`) until
        # the gripper_fixture sits within reach and at a graspable size.
        self.scene.debris = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/debris",
            spawn=UsdFileCfg(
                usd_path=SRB_ASSETS_DIR_SPACE.joinpath("debris_v3.usd").as_posix(),
                scale=(1.1, 1.1, 1.1),
                collision_props=CollisionPropertiesCfg(),
                ## NOTE: None of the mep USD layers author a mesh collision
                ## approximation, so PhysX would fall back to convexHull and turn
                ## the gripper_fixture handle into a solid blob the fingers cannot
                ## reach into. Decomposing it is what makes the handle graspable.
                mesh_collision_props=MeshCollisionPropertiesCfg(
                    mesh_approximation="convexDecomposition"
                ),
                rigid_props=RigidBodyPropertiesCfg(),
                ## NOTE: `mass` (not `density`) is used deliberately. UsdPhysics.MassAPI
                ## gives an authored `physics:mass` precedence over `physics:density`, so
                ## this pins the MEP's total mass to a fixed value regardless of its
                ## collision volume. With `density=1000.0` the convexDecomposition
                ## collision volume put the real mass at ~143,183 kg (measured via
                ## `root_physx_view.get_masses()`), which the Canadarm3's actuators
                ## (`effort_limit_sim=2500`, `stiffness=40000`) cannot move at a useful
                ## rate. 1000 kg is a manual-control testing value, not a physically
                ## accurate MEP mass -- revert to `density=1000.0` to restore it.
                mass_props=MassPropertiesCfg(mass=3000.0),
                activate_contact_sensors=True,
            ),
            ## NOTE: Pose taken from the reference stage `no_gripper.usd`, where the
            ## MEP was moved into the reach of the Canadarm3 (the previous pose was
            ## ~14.8 m from the flange, beyond the arm's ~8.5 m reach).
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(-0.802966, 11.409131, 8.408625),
                rot=(0.095277, -0.700659, 0.095277, -0.700659),
                lin_vel=(0.0, 0.0, 0.0),
                ang_vel=(0.0, 0.0, 0.0),
            ),
        )

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.debris.prim_path
            ]

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############


class Task(ManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._obj: RigidObject = self.scene["debris"]

        ## Capture
        self._capture: CaptureManager | None = (
            CaptureManager(
                self.cfg.capture,
                stage=self.scene.stage,
                env_prim_paths=self.scene.env_prim_paths,
                robot=self._robot,
                mep=self._obj,
                flange_offset=self._capture_flange_offset,
            )
            if self._capture_enabled
            else None
        )

    @property
    def _capture_enabled(self) -> bool:
        return self.cfg.capture.enable and self.cfg._robot.end_effector is None

    @property
    def _capture_flange_offset(self):
        flange = self.cfg._robot.frame_flange
        return flange.offset.pos, flange.offset.rot

    def _setup_scene(self):
        super()._setup_scene()

        ## MEP capture marker (see `TaskCfg.debris_marker_xform`)
        stage = self.scene.stage
        debris_name = self.cfg.scene.debris.prim_path.rsplit("/", 1)[-1]
        for env_prim_path in self.scene.env_prim_paths:
            debris_prim_path = f"{env_prim_path}/{debris_name}"
            if self.cfg.debris_marker_xform is not None:
                marker = stage.GetPrimAtPath(
                    f"{debris_prim_path}/{self.cfg.capture.marker_relpath}"
                )
                translate, orient, scale = self.cfg.debris_marker_xform
                marker.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*translate))
                marker.GetAttribute("xformOp:orient").Set(
                    Gf.Quatd(orient[0], *orient[1:])
                )
                marker.GetAttribute("xformOp:scale").Set(Gf.Vec3d(*scale))
            for relpath in self.cfg.debris_inactive_relpaths:
                stage.GetPrimAtPath(f"{debris_prim_path}/{relpath}").SetActive(False)

        ## Capture cylinder on the flange link (replaces the gripper)
        if self._capture_enabled:
            if self.cfg.capture.robot_link is None:
                self.cfg.capture.robot_link = self.cfg._robot.frame_flange.prim_relpath
            spawn_capture_cylinder(
                self.cfg.capture,
                f"{self.scene['robot'].cfg.prim_path}/{self.cfg.capture.robot_link}",
                self._capture_flange_offset,
            )

    def update_capture(self):
        """Evaluate the capture condition; call after every physics step."""
        if self._capture is not None:
            self._capture.update()

    def release_capture(self, env_ids: Sequence[int] | None = None):
        if self._capture is not None:
            self._capture.release(env_ids)

    def _get_dones(self):
        # Agents that go through `env.step()` get capture updates here, once per step
        self.update_capture()
        return super()._get_dones()

    def _reset_idx(self, env_ids: Sequence[int]):
        # Detach before the reset teleports the MEP and the arm, otherwise the fixed
        # joint would yank them back together
        self.release_capture(env_ids)
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        return _compute_step_return(
            ## Time
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            ## Actions
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            ## States
            # Joints
            joint_pos_robot=self._robot.data.joint_pos,
            joint_pos_limits_robot=(
                self._robot.data.soft_joint_pos_limits
                if torch.all(torch.isfinite(self._robot.data.soft_joint_pos_limits))
                else None
            ),
            joint_pos_end_effector=self._end_effector.data.joint_pos
            if isinstance(self._end_effector, Articulation)
            else None,
            joint_pos_limits_end_effector=(
                self._end_effector.data.soft_joint_pos_limits
                if isinstance(self._end_effector, Articulation)
                and torch.all(
                    torch.isfinite(self._end_effector.data.soft_joint_pos_limits)
                )
                else None
            ),
            joint_acc_robot=self._robot.data.joint_acc,
            joint_applied_torque_robot=self._robot.data.applied_torque,
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Transforms (world frame)
            tf_pos_end_effector=self._tf_end_effector.data.target_pos_w[:, 0, :],
            tf_quat_end_effector=self._tf_end_effector.data.target_quat_w[:, 0, :],
            tf_pos_obj=self._obj.data.root_com_pos_w,
            tf_quat_obj=self._obj.data.root_com_quat_w,
            # Object velocity
            vel_lin_obj=self._obj.data.root_com_lin_vel_w,
            vel_ang_obj=self._obj.data.root_com_ang_vel_w,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            contact_force_matrix_end_effector=self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
        )


@torch.jit.script
def _compute_step_return(
    *,
    ## Time
    episode_length: torch.Tensor,
    max_episode_length: int,
    truncate_episodes: bool,
    ## Actions
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    ## States
    # Joints
    joint_pos_robot: torch.Tensor,
    joint_pos_limits_robot: torch.Tensor | None,
    joint_pos_end_effector: torch.Tensor | None,
    joint_pos_limits_end_effector: torch.Tensor | None,
    joint_acc_robot: torch.Tensor,
    joint_applied_torque_robot: torch.Tensor,
    # Kinematics
    fk_pos_end_effector: torch.Tensor,
    fk_quat_end_effector: torch.Tensor,
    # Transforms (world frame)
    tf_pos_end_effector: torch.Tensor,
    tf_quat_end_effector: torch.Tensor,
    tf_pos_obj: torch.Tensor,
    tf_quat_obj: torch.Tensor,
    # Object velocity
    vel_lin_obj: torch.Tensor,
    vel_ang_obj: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    contact_force_matrix_end_effector: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Joints
    # Robot joints
    joint_pos_robot_normalized = (
        scale_transform(
            joint_pos_robot,
            joint_pos_limits_robot[:, :, 0],
            joint_pos_limits_robot[:, :, 1],
        )
        if joint_pos_limits_robot is not None
        else joint_pos_robot
    )
    # End-effector joints
    joint_pos_end_effector_normalized = (
        scale_transform(
            joint_pos_end_effector,
            joint_pos_limits_end_effector[:, :, 0],
            joint_pos_limits_end_effector[:, :, 1],
        )
        if joint_pos_end_effector is not None
        and joint_pos_limits_end_effector is not None
        else (
            joint_pos_end_effector
            if joint_pos_end_effector is not None
            else torch.empty((num_envs, 0), dtype=dtype, device=device)
        )
    )

    ## Kinematics
    fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
    fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

    ## Transforms (world frame)
    # End-effector -> Object
    tf_pos_end_effector_to_obj, tf_quat_end_effector_to_obj = subtract_frame_transforms(
        t01=tf_pos_end_effector,
        q01=tf_quat_end_effector,
        t02=tf_pos_obj,
        q02=tf_quat_obj,
    )
    tf_rotmat_end_effector_to_obj = matrix_from_quat(tf_quat_end_effector_to_obj)
    tf_rot6d_end_effector_to_obj = rotmat_to_rot6d(tf_rotmat_end_effector_to_obj)

    ## Contacts
    contact_forces_mean_robot = contact_forces_robot.mean(dim=1)
    contact_forces_mean_end_effector = (
        contact_forces_end_effector.mean(dim=1)
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )
    contact_forces_end_effector = (
        contact_forces_end_effector
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.5
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.mean(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Joint torque
    WEIGHT_JOINT_TORQUE = -0.000025
    MAX_JOINT_TORQUE_PENALTY = -4.0
    penalty_joint_torque = torch.clamp_min(
        WEIGHT_JOINT_TORQUE
        * torch.sum(torch.square(joint_applied_torque_robot), dim=1),
        min=MAX_JOINT_TORQUE_PENALTY,
    )

    # Penalty: Joint acceleration
    WEIGHT_JOINT_ACCELERATION = -0.0005
    MAX_JOINT_ACCELERATION_PENALTY = -4.0
    penalty_joint_acceleration = torch.clamp_min(
        WEIGHT_JOINT_ACCELERATION * torch.sum(torch.square(joint_acc_robot), dim=1),
        min=MAX_JOINT_ACCELERATION_PENALTY,
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ = 16.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ = 0.05
    reward_distance_end_effector_to_obj = WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_end_effector_to_obj, dim=-1)
            / TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ
        )
    )

    # Reward: Grasp object
    WEIGHT_GRASP = 16.0
    THRESHOLD_GRASP = 5.0
    reward_grasp = (
        WEIGHT_GRASP
        * (
            torch.mean(
                torch.max(
                    torch.norm(contact_force_matrix_end_effector, dim=-1), dim=-1
                )[0],
                dim=1,
            )
            > THRESHOLD_GRASP
        )
        if contact_force_matrix_end_effector is not None
        else torch.zeros(num_envs, dtype=dtype, device=device)
    )

    # Penalty: Debris velocity
    WEIGHT_DEBRIS_VELOCITY_LIN = -3.0
    WEIGHT_DEBRIS_VELOCITY_ANG = -1.0
    penalty_debris_velocity = WEIGHT_DEBRIS_VELOCITY_LIN * torch.norm(
        vel_lin_obj, dim=-1
    ) + WEIGHT_DEBRIS_VELOCITY_ANG * torch.norm(vel_ang_obj, dim=-1)

    ##################
    ## Terminations ##
    ##################
    # Termination: Debris too far from the end-effector
    termination_debris_too_far = torch.norm(tf_pos_end_effector_to_obj, dim=-1) > 10.0
    # Termination
    termination = termination_debris_too_far
    # Truncation
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        {
            "state": {
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                "tf_pos_end_effector_to_obj": tf_pos_end_effector_to_obj,
                "tf_rot6d_end_effector_to_obj": tf_rot6d_end_effector_to_obj,
                "vel_lin_obj": vel_lin_obj,
                "vel_ang_obj": vel_ang_obj,
            },
            "state_dyn": {
                "contact_forces_robot": contact_forces_robot,
                "contact_forces_end_effector": contact_forces_end_effector,
            },
            "proprio": {
                "fk_pos_end_effector": fk_pos_end_effector,
                "fk_rot6d_end_effector": fk_rot6d_end_effector,
            },
            "proprio_dyn": {
                "joint_pos_robot_normalized": joint_pos_robot_normalized,
                "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
                "joint_acc_robot": joint_acc_robot,
                "joint_applied_torque_robot": joint_applied_torque_robot,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_joint_torque": penalty_joint_torque,
            "penalty_joint_acceleration": penalty_joint_acceleration,
            "reward_distance_end_effector_to_obj": reward_distance_end_effector_to_obj,
            "reward_grasp": reward_grasp,
            "penalty_debris_velocity": penalty_debris_velocity,
        },
        termination,
        truncation,
    )
