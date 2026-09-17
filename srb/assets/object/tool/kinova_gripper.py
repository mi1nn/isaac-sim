from srb.core.action import (
    ActionGroup,
    BinaryJointPositionActionCfg,
    JointPositionBinaryActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ActiveTool, ArticulationCfg, Frame, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT
from .scaled_kinova import KINOVA_SCALE, KINOVA_TCP_DISTANCE, spawn_scaled_kinova

class Kinova300Large(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/kinova300_large",
        spawn=UsdFileCfg(
            func=spawn_scaled_kinova,
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("gripper")
            .joinpath("kinova300.usdz")
            .as_posix(),
            scale=(KINOVA_SCALE,) * 3,
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0,
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=0.5,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=32,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "kinova300_joint_finger_[1-3]": 0.2,
                "kinova300_joint_finger_tip_[1-3]": 0.2,
            },
        ),
        actuators={
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=[".*_finger_[1-3]", ".*_finger_tip_[1-3]"],
                velocity_limit_sim=1.0,
                effort_limit_sim=40.0,
                stiffness=400.0,
                damping=40.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = JointPositionBinaryActionGroup(
        BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "kinova300_joint_finger_[1-3]",
                "kinova300_joint_finger_tip_[1-3]",
            ],
            close_command_expr={
                "kinova300_joint_finger_[1-3]": 1.2,
                "kinova300_joint_finger_tip_[1-3]": 1.2,
            },
            open_command_expr={
                "kinova300_joint_finger_[1-3]": 0.2,
                "kinova300_joint_finger_tip_[1-3]": 0.2,
            },
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(rot=rpy_to_quat((180.0, 0.0, 0.0))),
    )
    frame_tool_centre_point: Frame = Frame(
        # USD traversal: native TCP is -0.16 m in the base rigid-link frame.
        prim_relpath="base", offset=Transform(pos=(0.0, 0.0, -KINOVA_TCP_DISTANCE))
    )



class Kinova300(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/kinova300",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("gripper")
            .joinpath("kinova300.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "kinova300_joint_finger_[1-3]": 0.2,
                "kinova300_joint_finger_tip_[1-3]": 0.2,
            },
        ),
        actuators={
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=[".*_finger_[1-3]", ".*_finger_tip_[1-3]"],
                velocity_limit=100.0,
                effort_limit=2.0,
                stiffness=1200.0,
                damping=10.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = JointPositionBinaryActionGroup(
        BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "kinova300_joint_finger_[1-3]",
                "kinova300_joint_finger_tip_[1-3]",
            ],
            close_command_expr={
                "kinova300_joint_finger_[1-3]": 1.2,
                "kinova300_joint_finger_tip_[1-3]": 1.2,
            },
            open_command_expr={
                "kinova300_joint_finger_[1-3]": 0.2,
                "kinova300_joint_finger_tip_[1-3]": 0.2,
            },
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(rot=rpy_to_quat((180.0, 0.0, 0.0))),
    )
    frame_tool_centre_point: Frame = Frame(
        prim_relpath="base", offset=Transform(pos=(0.0, 0.0, 0.16))
    )
