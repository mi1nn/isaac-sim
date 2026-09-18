"""P0 fixed-base scene for Canadarm3 + Kinova300Large and a stationary MEP."""
from pathlib import Path
import math

import torch
from isaacsim.core.utils.stage import get_current_stage
from pxr import Usd, UsdPhysics

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import RigidObjectCfg
from srb.core.domain import Domain
from srb.core.env import ManipulationEnv, ManipulationEnvCfg, ManipulationSceneCfg, ViewerCfg
from srb.core.sensor import ContactSensorCfg
from srb.core.sim import CollisionPropertiesCfg, MassPropertiesCfg, RigidBodyPropertiesCfg, UsdFileCfg
from srb.utils.cfg import configclass
from srb.utils.mep import configure_mep_manipulator
from srb.utils.path import SRB_ASSETS_DIR_SPACE
from .asset import spawn_mep


@configclass
class SceneCfg(ManipulationSceneCfg):
    num_envs: int = 1
    env_spacing: float = 40.0
    mep: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/mep",
        spawn=UsdFileCfg(
            func=spawn_mep,
            usd_path=str(Path(__file__).with_name("mep.usda")),
            activate_contact_sensors=True,
            rigid_props=RigidBodyPropertiesCfg(disable_gravity=True, max_depenetration_velocity=0.5),
            mass_props=MassPropertiesCfg(mass=100.0),
        ),
        # World/env frame, metres; quaternion order (w,x,y,z).
        # The handle is the root origin; the large MEP extends away from the arm.
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, 3.5, 5.0), rot=(0.0, 0.0, 0.0, 1.0)),
    )
    satellite: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/satellite",
        spawn=UsdFileCfg(
            usd_path=str(SRB_ASSETS_DIR_SPACE / "satellite.usd"),
            rigid_props=RigidBodyPropertiesCfg(disable_gravity=True, kinematic_enabled=True),
            collision_props=CollisionPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=1000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(15.0, 12.0, 0.0)),
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    domain: Domain = Domain.ORBIT
    robot = assets.Canadarm3(end_effector=assets.Kinova300Large())
    scenery = None
    pedestal = None
    scene: SceneCfg = SceneCfg()
    episode_length_s: float = 120.0
    truncate_episodes: bool = False
    debug_vis: bool = True
    # Simulation joint speed limit, rad/s. Damping controls actual drive response.
    arm_velocity_limit: float = 0.5
    arm_damping: float = 5000.0
    ik_action_scale: float = 0.1
    viewer: ViewerCfg = ViewerCfg(eye=(13.0, -15.0, 13.0), lookat=(1.5, 3.0, 4.0), origin_type="env")

    def __post_init__(self):
        super().__post_init__()
        if not isinstance(self._robot, assets.Canadarm3) or not isinstance(self._robot.end_effector, assets.Kinova300Large):
            raise ValueError("P0 satellite_mission requires Canadarm3 + Kinova300Large")
        for name in ("arm_velocity_limit", "arm_damping", "ik_action_scale"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        configure_mep_manipulator(self)
        actuator = self.scene.robot.actuators["joints"]
        actuator.velocity_limit = self.arm_velocity_limit
        actuator.velocity_limit_sim = self.arm_velocity_limit
        actuator.damping = self.arm_damping
        getattr(self.actions, "robot/differential_inverse_kinematics").scale = self.ik_action_scale
        self.events.randomize_robot_joints = None
        self.events.randomize_skydome_orientation = None
        self.events.randomize_gravity = None
        self.scene.contacts_end_effector = ContactSensorCfg(
            prim_path=f"{self.scene.end_effector.prim_path}/.*",
            filter_prim_paths_expr=[self.scene.mep.prim_path],
            debug_vis=self.debug_vis,
        )


class Task(ManipulationEnv):
    cfg: TaskCfg

    def _setup_scene(self):
        super()._setup_scene()
        stage = get_current_stage()
        for i in range(self.num_envs):
            root = stage.GetPrimAtPath(f"/World/envs/env_{i}/mep")
            bodies = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(bodies) != 1 or bodies[0] != root:
                raise ValueError(f"Expected one MEP root rigid body, got {[str(p.GetPath()) for p in bodies]}")
            if not stage.GetPrimAtPath(f"/World/envs/env_{i}/mep/grasp_frame"):
                raise ValueError("MEP grasp_frame is missing")

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        # The common reset restores articulations only. Restore both mission bodies too.
        for name in ("mep", "satellite"):
            body = self.scene[name]
            state = body.data.default_root_state[env_ids].clone()
            state[:, :3] += self.scene.env_origins[env_ids]
            body.write_root_pose_to_sim(state[:, :7], env_ids=env_ids)
            if not body.cfg.spawn.rigid_props.kinematic_enabled:
                body.write_root_velocity_to_sim(state[:, 7:], env_ids=env_ids)
            body.reset(env_ids)

    def extract_step_return(self) -> StepReturn:
        mep = self.scene["mep"]
        zero = torch.zeros(self.num_envs, device=self.device)
        done = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return StepReturn(
            {"state": {
                "mep_position_w": mep.data.root_pos_w,
                "mep_velocity_w": mep.data.root_vel_w,
                "tcp_position_b": self._tf_end_effector.data.target_pos_source[:, 0],
                "joint_position": self._robot.data.joint_pos,
                "finger_position": self._end_effector.data.joint_pos,
                "gripper_contact_forces_w": self._contacts_end_effector.data.net_forces_w,
            }},
            {"p0": zero}, done,
            self.episode_length_buf >= self.max_episode_length if self.cfg.truncate_episodes else done.clone(),
        )
