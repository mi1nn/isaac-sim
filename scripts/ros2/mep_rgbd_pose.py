#!/usr/bin/env python3
"""RGB-D + HSV (or external SAM2 mask) -> /mep/grasp_pose. No motor commands."""
import time

import cv2
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from grasp_geometry import backproject, grasp_from_points

QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def image_array(msg):
    formats = {"rgb8": (np.uint8, 3), "rgba8": (np.uint8, 4),
               "mono8": (np.uint8, 1), "32FC1": (np.float32, 1)}
    if msg.encoding not in formats:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    dtype, channels = formats[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    row_bytes = msg.width*channels*dtype.itemsize
    if msg.step < row_bytes or len(msg.data) < msg.step*msg.height:
        raise ValueError("Incomplete image buffer")
    raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    result = np.ascontiguousarray(raw[:, :row_bytes]).view(dtype)
    return result.reshape(msg.height, msg.width, channels) if channels > 1 else result.reshape(msg.height, msg.width)


class MepRgbdPose(Node):
    def __init__(self):
        super().__init__("mep_rgbd_pose", parameter_overrides=[Parameter("use_sim_time", value=True)])
        defaults = dict(camera_ns="/srb/env0/cam_wrist", output_topic="/mep/grasp_pose",
                        base_frame="mrv/base_link", mask_topic="/perception/wrist/target_mask",
                        mask_source="hsv", min_pixels=12, max_depth=30.0,
                        max_image_age=0.5, processing_rate=20.0, smoothing=0.4,
                        handle_radius=0.075, hsv_lower=[45, 120, 60], hsv_upper=[75, 255, 255])
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.g = lambda key: self.get_parameter(key).value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.output = self.create_publisher(PoseStamped, self.g("output_topic"), QOS)
        self.debug = self.create_publisher(Image, "~/debug_image", QOS)
        ns = self.g("camera_ns")
        self.inputs = [message_filters.Subscriber(self, kind, f"{ns}/{suffix}", qos_profile=QOS)
                       for kind, suffix in ((Image, "image_rgb"), (Image, "image_depth"), (CameraInfo, "camera_info"))]
        if self.g("mask_source") == "external":
            self.inputs.append(message_filters.Subscriber(self, Image, self.g("mask_topic"), qos_profile=QOS))
        elif self.g("mask_source") != "hsv":
            raise ValueError("mask_source must be hsv or external")
        # Exact acquisition stamps, including any externally computed mask. A SAM2
        # producer must retain the RGB header, not replace it with inference time.
        self.sync = message_filters.TimeSynchronizer(self.inputs, 30)
        self.sync.registerCallback(self.receive)
        self.pending = None
        self.previous = None
        self.axis = None
        self.last_stamp = -1
        self.create_timer(1/self.g("processing_rate"), self.process)

    def receive(self, *messages):
        self.pending = messages

    def process(self):
        if self.pending is None:
            return
        rgb, depth, info, *external = self.pending
        stamp = Time.from_msg(rgb.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        if age < 0 or age > self.g("max_image_age"):
            self.pending = None
            self.previous = self.axis = None
            return
        if stamp.nanoseconds == self.last_stamp:
            return
        try:
            transform = self.tf.lookup_transform(self.g("base_frame"), rgb.header.frame_id, stamp)
        except TransformException:
            return  # TF callback can arrive after the images; retry this tuple.
        self.pending = None
        self.last_stamp = stamp.nanoseconds
        try:
            if not (rgb.header.frame_id == depth.header.frame_id == info.header.frame_id):
                raise ValueError("RGB/depth/camera_info optical frames must match")
            color, distances = image_array(rgb)[..., :3], image_array(depth)
            if external:
                if external[0].header.frame_id != rgb.header.frame_id:
                    raise ValueError("External mask must keep the RGB optical frame")
                mask = image_array(external[0]) > 0
            else:
                mask = cv2.inRange(cv2.cvtColor(color, cv2.COLOR_RGB2HSV),
                                   np.array(self.g("hsv_lower")), np.array(self.g("hsv_upper")))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
                mask = labels == (1+np.argmax(stats[1:, cv2.CC_STAT_AREA])) if count > 1 else np.zeros_like(mask, bool)
            points = backproject(mask, distances, np.array(info.k).reshape(3, 3),
                                 self.g("min_pixels"), self.g("max_depth"))
            overlay = color.copy()
            overlay[mask > 0] = (255, 0, 255)
            debug = Image(header=rgb.header, height=rgb.height, width=rgb.width,
                          encoding="rgb8", step=rgb.width*3, data=overlay.tobytes())
            self.debug.publish(debug)
            if points is None:
                self.previous = self.axis = None
                return
            t, q = transform.transform.translation, transform.transform.rotation
            origin = np.array([t.x, t.y, t.z])
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
            estimate = grasp_from_points(rotation.apply(points)+origin, origin, self.axis, self.g("handle_radius"))
            if estimate is None:
                return
            position, quaternion, self.axis = estimate
            if self.previous is not None:
                a = self.g("smoothing")
                position = (1-a)*self.previous+a*position
            self.previous = position
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = rgb.header.stamp, self.g("base_frame")
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quaternion)
            self.output.publish(msg)
        except (ValueError, IndexError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = MepRgbdPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
