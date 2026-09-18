#!/usr/bin/env python3
"""PoseStamped -> relative pose IK / gripper, independent of perception backend."""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from grasp_geometry import limited, pose_error


class MepPoseController(Node):
    def __init__(self):
        super().__init__("mep_pose_controller", parameter_overrides=[Parameter("use_sim_time", value=True)])
        defaults = dict(pose_topic="/mep/grasp_pose", base_frame="mrv/base_link", tcp_frame="mrv/tcp",
                        twist_topic="/srb/env0/robot/robot/differential_inverse_kinematics",
                        gripper_topic="/srb/env0/end_effector/end_effector/binary_joint_position",
                        control_rate=20.0, sim_step=0.02, action_scale=0.1, max_pose_age=0.5,
                        wall_timeout=1.0, standoff=0.6, kp_position=0.8, kp_orientation=0.8,
                        max_speed=0.15, final_speed=0.04, max_angular_speed=0.15,
                        position_tolerance=0.03, angle_tolerance=0.10, close_duration=3.0,
                        search_enabled=False, search_speed=0.08, search_duration=30.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.g = lambda key: self.get_parameter(key).value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.twist = self.create_publisher(Twist, self.g("twist_topic"), qos)
        self.gripper = self.create_publisher(Bool, self.g("gripper_topic"), qos)
        self.status = self.create_publisher(String, "~/state", qos)
        self.create_subscription(PoseStamped, self.g("pose_topic"), self.receive, qos)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pose = None
        self.received = 0.0
        self.last_stamp = -1
        self.started = None
        self.state = "WAIT_POSE"
        self.closed_since = None
        self.ever_detected = False
        self.create_timer(1/self.g("control_rate"), self.step)

    def receive(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        p, q = msg.pose.position, msg.pose.orientation
        values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        if msg.header.frame_id != self.g("base_frame") or not np.isfinite(values).all():
            return
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        if age < 0 or age > self.g("max_pose_age") or np.linalg.norm(values[3:]) < 1e-6:
            return
        # Duplicate messages cannot keep an old estimate alive.
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.pose = (values[:3], values[3:]/np.linalg.norm(values[3:]), stamp)
        self.received = time.monotonic()
        self.ever_detected = True

    def set_state(self, state):
        if self.state != state:
            self.get_logger().info(f"{self.state} -> {state}")
            self.state = state

    def step(self):
        now = self.get_clock().now()
        if self.started is None:
            self.started = now.nanoseconds
        command = Twist()
        closed = self.state in ("CLOSE", "HOLD_UNVERIFIED")
        try:
            tf = self.tf.lookup_transform(self.g("base_frame"), self.g("tcp_frame"), Time())
            tf_age = (now-Time.from_msg(tf.header.stamp)).nanoseconds*1e-9
            if tf_age < 0 or tf_age > self.g("max_pose_age"):
                raise ValueError("stale TCP transform")
            p, q = tf.transform.translation, tf.transform.rotation
            current_p, current_q = np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])
            fresh = self.pose is not None and 0 <= (now.nanoseconds-self.pose[2])*1e-9 <= self.g("max_pose_age") and time.monotonic()-self.received <= self.g("wall_timeout")
            if closed:
                if self.state == "CLOSE" and (now.nanoseconds-self.closed_since)*1e-9 >= self.g("close_duration"):
                    # Closing is not proof of a physical grasp; a contact/retention
                    # check is deliberately not replaced with a timer-based success.
                    self.set_state("HOLD_UNVERIFIED")
            elif not fresh:
                self.set_state("WAIT_POSE")
                if self.g("search_enabled") and not self.ever_detected and (now.nanoseconds-self.started)*1e-9 < self.g("search_duration"):
                    command.angular.z = self.g("search_speed")*self.g("sim_step")/self.g("action_scale")
            else:
                target_p, target_q, _ = self.pose
                direction = -Rotation.from_quat(target_q).as_matrix()[:, 2]
                if self.state == "WAIT_POSE":
                    self.set_state("APPROACH")
                goal = target_p-direction*self.g("standoff") if self.state == "APPROACH" else target_p
                dp, dr = pose_error(goal, target_q, current_p, current_q)
                speed = self.g("max_speed") if self.state == "APPROACH" else self.g("final_speed")
                v = limited(dp*self.g("kp_position"), speed)
                w = limited(dr*self.g("kp_orientation"), self.g("max_angular_speed"))
                # SRB Twist transports a dimensionless relative-pose action,
                # NOT m/s. The latched action is applied every simulator step.
                factor = self.g("sim_step")/self.g("action_scale")
                command.linear.x, command.linear.y, command.linear.z = map(float, v*factor)
                command.angular.x, command.angular.y, command.angular.z = map(float, w*factor)
                if np.linalg.norm(dp) < self.g("position_tolerance") and np.linalg.norm(dr) < self.g("angle_tolerance"):
                    if self.state == "APPROACH":
                        self.set_state("FINAL")
                    else:
                        self.set_state("CLOSE")
                        self.closed_since = now.nanoseconds
                        closed = True
                        command = Twist()
        except (TransformException, ValueError):
            pass  # Always publish zero, never leave the previous motion latched.
        self.twist.publish(command)
        self.gripper.publish(Bool(data=closed))
        self.status.publish(String(data=self.state))

    def stop(self):
        self.twist.publish(Twist())


def main():
    rclpy.init()
    node = MepPoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
