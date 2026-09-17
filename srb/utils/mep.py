"""Project-specific frame setup for the Canadarm3 / enlarged Kinova assembly."""

from srb.assets.object.tool.scaled_kinova import KINOVA_TCP_DISTANCE
from srb.utils.math import combine_frame_transforms_tuple


def configure_mep_manipulator(cfg):
    from srb.assets.object.tool.kinova_gripper import Kinova300Large
    from srb.assets.robot.manipulation.canadarm3 import Canadarm3

    robot = cfg._robot
    if not isinstance(robot, Canadarm3) or not isinstance(robot.end_effector, Kinova300Large):
        return
    tool = robot.end_effector
    # T_link7_tcp = T_link7_flange * T_flange_base * T_base_tcp.
    # All quaternions here are Isaac/Usd (w,x,y,z), translations are metres.
    assembly = cfg.joint_assemblies["end_effector"]
    tcp_pos, tcp_rot = combine_frame_transforms_tuple(
        assembly.fixed_joint_offset, assembly.fixed_joint_orient,
        tool.frame_tool_centre_point.offset.pos, tool.frame_tool_centre_point.offset.rot,
    )
    action = getattr(cfg.actions, "robot/differential_inverse_kinematics")
    action.body_offset.pos, action.body_offset.rot = tcp_pos, tcp_rot
    # Sensor TCP is measured from the physical gripper base, not the flange.
    target = cfg.scene.tf_end_effector.target_frames[0]
    target.prim_path = f"{cfg.scene.end_effector.prim_path}/base"
    target.offset.pos = (0.0, 0.0, -KINOVA_TCP_DISTANCE)
    target.offset.rot = (1.0, 0.0, 0.0, 0.0)
    # Repeatable startup; random joint offsets can start in contact with scenery.
    cfg.events.randomize_robot_joints = None
    cfg.mep_ros_frames = True
