#!/usr/bin/env python3
"""Bridge from the real satellite docking CSV to ROS2 monitoring topics.

Source CSV: feature/docking -> *_docking.csv

Quaternion source convention:
    (w, x, y, z)

Displayed angular velocity:
    relative angular speed between PROBE_DOCK_POINT and SAT_DOCK_POINT.
"""

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32, String


TOTAL_MISSION_STEPS = 7

DOCKING_ACTIVE_STATES = {
    "DOCK_TARGET_ACQUIRE",
    "PRE_DOCK_APPROACH",
    "XY_ALIGN",
    "ORIENTATION_ALIGN",
    "ALIGNMENT_CHECK",
    "Z_APPROACH",
    "FINAL_INSERTION",
    "DOCK_READY",
}

DOCKED_STATES = {
    "DOCKED",
    "DOCK_HOLDING",
    "SUCCESS",
}


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} is not finite")
    return value


def _normalize_quat_wxyz(q):
    q = tuple(float(v) for v in q)
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        raise ValueError("zero quaternion")
    return tuple(v / n for v in q)


def _quat_conjugate_wxyz(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_multiply_wxyz(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def relative_quaternion(row):
    """Return q_dock^-1 * q_probe."""
    probe = _normalize_quat_wxyz(
        (
            row["probe_tip_qw"],
            row["probe_tip_qx"],
            row["probe_tip_qy"],
            row["probe_tip_qz"],
        )
    )

    dock = _normalize_quat_wxyz(
        (
            row["sat_dock_qw"],
            row["sat_dock_qx"],
            row["sat_dock_qy"],
            row["sat_dock_qz"],
        )
    )

    return _normalize_quat_wxyz(
        _quat_multiply_wxyz(
            _quat_conjugate_wxyz(dock),
            probe,
        )
    )


def relative_angular_speed_deg_s(prev_q, q, dt):
    """Shortest relative orientation change divided by dt."""
    if prev_q is None or dt is None or dt <= 0.0:
        return 0.0

    a = _normalize_quat_wxyz(prev_q)
    b = _normalize_quat_wxyz(q)

    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = max(0.0, min(1.0, dot))

    angle_rad = 2.0 * math.acos(dot)

    return math.degrees(angle_rad) / dt


def progress_for_state(state):
    if state in DOCKED_STATES:
        return 5

    if state in DOCKING_ACTIVE_STATES:
        return 4

    if state == "DOCK_FAILED":
        return 4

    return 4


def docking_metrics(row, prev_q=None, prev_timestamp=None):
    timestamp = _finite(row["timestamp"], "timestamp")

    ex = _finite(row["relative_x"], "relative_x")
    ey = _finite(row["relative_y"], "relative_y")
    ez = _finite(row["relative_z"], "relative_z")

    position_error = math.sqrt(
        ex * ex + ey * ey + ez * ez
    )

    total_velocity = _finite(
        row["relative_speed_mps"],
        "relative_speed_mps",
    )

    orientation_error = _finite(
        row["orientation_error_deg"],
        "orientation_error_deg",
    )

    remaining_distance = _finite(
        row["geometry_distance_m"],
        "geometry_distance_m",
    )

    q_rel = relative_quaternion(row)

    dt = None
    if prev_timestamp is not None:
        dt = timestamp - prev_timestamp

    total_angular_velocity = (
        relative_angular_speed_deg_s(
            prev_q,
            q_rel,
            dt,
        )
    )

    return {
        "timestamp": timestamp,
        "state": row["state"],
        "position_error": position_error,
        "orientation_error": orientation_error,
        "total_velocity": total_velocity,
        "total_angular_velocity": total_angular_velocity,
        "remaining_distance": remaining_distance,
        "progress_current": progress_for_state(row["state"]),
        "progress_total": TOTAL_MISSION_STEPS,
        "dock_success": str(row.get("dock_success", "0")).strip() == "1",
        "relative_q": q_rel,
    }


class DockingMetricsBridge(Node):

    def __init__(self):
        super().__init__("docking_metrics_bridge")

        self.csv_path = Path(
            self.declare_parameter(
                "csv_path",
                "",
            ).value
        ).expanduser()

        self.poll_sec = float(
            self.declare_parameter(
                "poll_sec",
                0.2,
            ).value
        )

        base = "/mep_monitor/docking"

        self.pub_position = self.create_publisher(
            Float64,
            f"{base}/position_error",
            10,
        )

        self.pub_orientation = self.create_publisher(
            Float64,
            f"{base}/orientation_error",
            10,
        )

        self.pub_velocity = self.create_publisher(
            Float64,
            f"{base}/total_velocity",
            10,
        )

        self.pub_angular_velocity = self.create_publisher(
            Float64,
            f"{base}/total_angular_velocity",
            10,
        )

        self.pub_remaining = self.create_publisher(
            Float64,
            f"{base}/remaining_distance",
            10,
        )

        self.pub_state = self.create_publisher(
            String,
            f"{base}/current_state",
            10,
        )

        self.pub_progress = self.create_publisher(
            String,
            f"{base}/progress",
            10,
        )

        self.pub_progress_current = self.create_publisher(
            Int32,
            f"{base}/progress_current",
            10,
        )

        self.pub_progress_total = self.create_publisher(
            Int32,
            f"{base}/progress_total",
            10,
        )

        self.pub_success = self.create_publisher(
            Bool,
            f"{base}/dock_success",
            10,
        )

        self.pub_valid = self.create_publisher(
            Bool,
            f"{base}/metrics_valid",
            10,
        )

        self.last_timestamp = None
        self.prev_timestamp = None
        self.prev_relative_q = None
        self.last_state = None
        self.last_warning = None

        self.create_timer(
            self.poll_sec,
            self.publish_latest,
        )

        self.get_logger().info(
            "Docking metrics bridge ready."
        )

        if str(self.csv_path):
            self.get_logger().info(
                f"CSV: {self.csv_path}"
            )
        else:
            self.get_logger().warning(
                "csv_path is empty. "
                "Pass -p csv_path:=/path/to/..._docking.csv"
            )

    def reject(self, reason):
        self.pub_valid.publish(
            Bool(data=False)
        )

        if reason != self.last_warning:
            self.last_warning = reason
            self.get_logger().warning(reason)

    def read_latest_row(self):
        if not str(self.csv_path):
            return None

        if not self.csv_path.is_file():
            self.reject(
                f"Docking CSV not found: {self.csv_path}"
            )
            return None

        last = None

        with self.csv_path.open(
            "r",
            newline="",
            encoding="utf-8",
        ) as f:
            reader = csv.DictReader(f)

            for row in reader:
                last = row

        return last

    def publish_latest(self):
        try:
            row = self.read_latest_row()

            if row is None:
                return

            timestamp = _finite(
                row["timestamp"],
                "timestamp",
            )

            if timestamp == self.last_timestamp:
                return

            metrics = docking_metrics(
                row,
                prev_q=self.prev_relative_q,
                prev_timestamp=self.prev_timestamp,
            )

            self.pub_position.publish(
                Float64(data=metrics["position_error"])
            )

            self.pub_orientation.publish(
                Float64(data=metrics["orientation_error"])
            )

            self.pub_velocity.publish(
                Float64(data=metrics["total_velocity"])
            )

            self.pub_angular_velocity.publish(
                Float64(
                    data=metrics["total_angular_velocity"]
                )
            )

            self.pub_remaining.publish(
                Float64(data=metrics["remaining_distance"])
            )

            self.pub_state.publish(
                String(data=metrics["state"])
            )

            progress_text = (
                f'{metrics["progress_current"]}'
                f'/{metrics["progress_total"]}'
            )

            self.pub_progress.publish(
                String(data=progress_text)
            )

            self.pub_progress_current.publish(
                Int32(data=metrics["progress_current"])
            )

            self.pub_progress_total.publish(
                Int32(data=metrics["progress_total"])
            )

            self.pub_success.publish(
                Bool(data=metrics["dock_success"])
            )

            self.pub_valid.publish(
                Bool(data=True)
            )

            self.prev_relative_q = metrics["relative_q"]
            self.prev_timestamp = metrics["timestamp"]
            self.last_timestamp = metrics["timestamp"]
            self.last_warning = None

            if metrics["state"] != self.last_state:
                self.get_logger().info(
                    f'STATE={metrics["state"]} | '
                    f'PROGRESS={progress_text} | '
                    f'POS={metrics["position_error"]:.4f} m | '
                    f'VEL={metrics["total_velocity"]:.4f} m/s | '
                    f'ANG={metrics["total_angular_velocity"]:.3f} deg/s | '
                    f'REMAIN={metrics["remaining_distance"]:.4f} m'
                )
                self.last_state = metrics["state"]

        except Exception as exc:
            self.reject(
                f"Docking metrics rejected: {exc}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = DockingMetricsBridge()
        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        rclpy.executors.ExternalShutdownException,
    ):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
