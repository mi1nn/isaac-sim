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
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Kinova300Large(ActiveTool):
    """Kinova 3-finger gripper scaled up 4.2x so that it can wrap the MEP handle.

    IMPORTANT: `scale` only grows the *geometry*. `kinova300.usdz` authors explicit
    `physics:mass` (base 0.99 kg, each finger / finger tip 0.01 kg) and explicit
    `physics:diagonalInertia` (finger tip: 7.9e-7 kg m^2), and in UsdPhysics an
    authored mass always takes precedence over a density. The scaled-up gripper
    therefore still has the mass and inertia of the 1x gripper, so the actuator
    gains have to be sized for THAT, not for the 4.2^3 = 74x larger volume.
    """

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/kinova300_large",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("gripper")
            .joinpath("kinova300.usdz")
            .as_posix(),
            scale=(4.2, 4.2, 4.2),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.021, rest_offset=0.0      # 0.005 × 4.2
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            ## NOTE: `mass_props=MassPropertiesCfg(density=1000.0)` used to sit here to
            ## "recompute the mass from the scaled volume". It never did anything: the
            ## USD authors `physics:mass` on every link and authored mass wins over
            ## density, so the links kept their 1x mass either way. It is dropped
            ## rather than kept, because it made the gains below look justified.
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                ## NOTE: 0 velocity iterations means PhysX never corrects the velocities
                ## the position solver produces, so one bad step is never damped out.
                solver_velocity_iteration_count=1,
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
                ## NOTE: These used to be stiffness=1_200_000 / damping=10_000 /
                ## effort=2000, i.e. the 1x gripper's gains multiplied by ~1000 for a
                ## mass increase that never happened (see the class docstring). On a
                ## 0.01 kg finger with I = 7.9e-7 kg m^2, a single clamped 2000 Nm
                ## impulse is dw = 2000 * (1/150) / 7.9e-7 ~ 1.7e7 rad/s -- the drive
                ## alone is enough to blow the articulation apart, and the reaction
                ## torque travels through the assembler's fixed joint into the arm.
                ## The values below keep a firm grip (50 Nm over a 4.2 x 0.044 m
                ## finger is ~270 N of tip force) while `velocity_limit_sim` gives
                ## PhysX a hard ceiling that makes divergence impossible.
                effort_limit_sim=50.0,
                velocity_limit_sim=5.0,
                stiffness=5000.0,
                damping=100.0,
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
    ## NOTE: `frame_mount` rotates the gripper by 180 deg about X and the Canadarm3
    ## flange rotates by 180 deg about Y. Composed, that is a 180 deg rotation about
    ## Z (Ry(180) @ Rx(180) = Rz(180)), which leaves the mount Z axis aligned with the
    ## flange Z axis -- exactly what is needed, because the gripper's geometry grows
    ## along -Z of `base` and the flange face is at -Z of `canadarm3_large_7`. The
    ## residual 180 deg twist only decides which of the 3 fingers points where.
    frame_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(rot=rpy_to_quat((180.0, 0.0, 0.0))),
    )
    frame_tool_centre_point: Frame = Frame(
        prim_relpath="base",
        offset=Transform(pos=(0.0, 0.0, 0.672)),  # 0.16 × 4.2 (was 0.64 = 0.16 × 4)
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
