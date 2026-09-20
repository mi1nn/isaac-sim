#!/usr/bin/env python3

import csv
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool


CSV_PATH = "/home/rokey/isaac_space/project/logs/vision_capture/capture_metrics.csv"


class IsaacMetricsBridge(Node):
    def __init__(self):
        super().__init__("isaac_metrics_bridge")

        self.pub_position = self.create_publisher(
            Float64, "/mep_monitor/actual/position_error", 10)
        self.pub_orientation = self.create_publisher(
            Float64, "/mep_monitor/actual/orientation_error", 10)
        self.pub_velocity = self.create_publisher(
            Float64, "/mep_monitor/actual/relative_velocity", 10)

        self.pub_state = self.create_publisher(
            String, "/mep_monitor/actual/capture_state", 10)
        self.pub_attached = self.create_publisher(
            Bool, "/mep_monitor/actual/capture_success", 10)

        self.last_timestamp = None

        self.get_logger().info(
            f"Reading REAL Isaac metrics from: {CSV_PATH}"
        )

    def publish_latest(self):
        try:
            with open(CSV_PATH, newline="") as f:
                rows = list(csv.DictReader(f))

            if not rows:
                return

            row = rows[-1]

            timestamp = row.get("timestamp")
            if timestamp == self.last_timestamp:
                return

            self.last_timestamp = timestamp

            position = float(row["ee_to_cylinder_distance_m"])
            orientation = float(row["ee_normal_angle_deg"])
            velocity = float(row["gt_relative_velocity_mps"])

            if not all(math.isfinite(x) for x in
                       (position, orientation, velocity)):
                return

            msg = Float64()

            msg.data = position
            self.pub_position.publish(msg)

            msg = Float64()
            msg.data = orientation
            self.pub_orientation.publish(msg)

            msg = Float64()
            msg.data = velocity
            self.pub_velocity.publish(msg)

            state = String()
            state.data = row["capture_state"]
            self.pub_state.publish(state)

            success = Bool()
            success.data = row["capture_success"] == "1"
            self.pub_attached.publish(success)

            self.get_logger().info(
                f"{row['capture_state']} | "
                f"pos={position:.4f} m | "
                f"ori={orientation:.3f} deg | "
                f"vel={velocity:.4f} m/s"
            )

        except Exception as e:
            self.get_logger().warning(str(e))


def main():
    rclpy.init()
    node = IsaacMetricsBridge()

    try:
        while rclpy.ok():
            node.publish_latest()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
