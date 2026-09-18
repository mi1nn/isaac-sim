#!/usr/bin/env python3
"""Passive ROS recorder. Never publishes motor commands; writes test evidence."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from mep_rgbd_pose import image_array


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--output", default="/tmp/mep_ros_report.json")
    args = parser.parse_args()
    rclpy.init()
    node = Node("mep_passive_check")
    start = time.monotonic()
    counts, latest, transforms, joints, poses, states = {}, {}, {}, [], [], []
    clock_times = []

    def callback(name, msg):
        counts[name] = counts.get(name, 0)+1
        latest[name] = msg
        if name == "tf":
            for tf in msg.transforms:
                t, q = tf.transform.translation, tf.transform.rotation
                transforms[tf.child_frame_id] = dict(parent=tf.header.frame_id, position=[t.x,t.y,t.z], quaternion=[q.x,q.y,q.z,q.w])
        elif name == "joints":
            joints.append(dict(wall_time=time.monotonic()-start, stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                               position=list(msg.position), velocity=list(msg.velocity)))
        elif name == "pose":
            p, q = msg.pose.position, msg.pose.orientation
            poses.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                              frame=msg.header.frame_id, position=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w]))
        elif name == "state" and (not states or states[-1][1] != msg.data):
            states.append([time.monotonic()-start, msg.data])
        elif name == "clock":
            clock_times.append(msg.clock.sec+msg.clock.nanosec*1e-9)

    for name, kind, topic in (
        ("rgb", Image, "/srb/env0/cam_wrist/image_rgb"),
        ("depth", Image, "/srb/env0/cam_wrist/image_depth"),
        ("info", CameraInfo, "/srb/env0/cam_wrist/camera_info"),
        ("tf", TFMessage, "/tf"), ("clock", Clock, "/clock"),
        ("joints", JointState, "/srb/env0/robot/joint_states"),
        ("gripper", JointState, "/srb/env0/end_effector/joint_states"),
        ("pose", PoseStamped, "/mep/grasp_pose"),
        ("state", String, "/mep_pose_controller/state"),
        ("twist", Twist, "/srb/env0/robot/robot/differential_inverse_kinematics"),
    ):
        node.create_subscription(kind, topic, lambda msg, key=name: callback(key, msg), 10)
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.1)
    summary = dict(counts=counts, wall_seconds=time.monotonic()-start,
                   simulation_seconds=clock_times[-1]-clock_times[0] if len(clock_times)>1 else 0,
                   transforms=transforms, joints=joints, poses=poses, states=states)
    if "rgb" in latest:
        color = image_array(latest["rgb"])[..., :3]
        cv2.imwrite(str(Path(args.output).with_suffix(".png")), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
        summary["image_shape"] = list(color.shape)
    if "depth" in latest:
        depth = image_array(latest["depth"])
        summary["depth_finite_fraction"] = float(np.isfinite(depth).mean())
        finite = depth[np.isfinite(depth)]
        summary["depth_range"] = [float(finite.min()), float(finite.max())] if len(finite) else []
    if "gripper" in latest:
        summary["fingers"] = list(latest["gripper"].position)
    if "twist" in latest:
        cmd = latest["twist"]
        summary["last_twist"] = [cmd.linear.x,cmd.linear.y,cmd.linear.z,cmd.angular.x,cmd.angular.y,cmd.angular.z]
    Path(args.output).write_text(json.dumps(summary, indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k not in ("transforms", "joints", "poses")}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
