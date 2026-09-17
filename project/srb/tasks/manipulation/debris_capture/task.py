from dataclasses import MISSING
from typing import Sequence

import torch

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
            scale=(2.0, 2.0, 2.0),
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(3.356, 26.3265, 11.4938),
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
    pedestal: Object | AssetVariant | None = None

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = True

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
                scale=(1.5, 1.5, 1.5),
                collision_props=CollisionPropertiesCfg(),
                ## NOTE: None of the mep USD layers author a mesh collision
                ## approximation, so PhysX would fall back to convexHull and turn
                ## the gripper_fixture handle into a solid blob the fingers cannot
                ## reach into. Decomposing it is what makes the handle graspable.
                mesh_collision_props=MeshCollisionPropertiesCfg(
                    mesh_approximation="convexDecomposition"
                ),
                rigid_props=RigidBodyPropertiesCfg(),
                mass_props=MassPropertiesCfg(density=1000.0),
                activate_contact_sensors=True,
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(-0.105728, 13.90236, 11.82852),
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

    def _reset_idx(self, env_ids: Sequence[int]):
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
