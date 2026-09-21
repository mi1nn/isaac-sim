#!/usr/bin/env python3
"""Actual Isaac MEP capture CSV -> ROS2 monitoring bridge."""

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


DEFAULT_CSV_PATH = (
    "/home/rokey/isaac_space/project/"
    "logs/vision_capture/capture_metrics.csv"
)

# feature/docking의 현재 final_gap_m
DEFAULT_CAPTURE_TARGET_GAP_M = 0.05


def _finite(value, name):
    value = float(value)

    if not math.isfinite(value):
        raise ValueError(f"{name} is not finite")

    return value


def _bool_value(value):
    return str(value).strip().lower() in {
        "1",
        "1.0",
        "true",
        "yes",
    }


def _first_finite(row, names):
    for name in names:
        value = row.get(name)

        if value in (None, ""):
            continue

        try:
            value = float(value)

            if math.isfinite(value):
                return value

        except ValueError:
            pass

    raise ValueError(
        "No finite value for: "
        + ", ".join(names)
    )


def _relative_angular_velocity_rad_s(row):
    value = row.get(
        "gt_relative_angular_velocity_rad_s"
    )

    if value not in (None, ""):
        try:
            value = float(value)

            if math.isfinite(value):
                return abs(value)

        except ValueError:
            pass

    ee = [
        _finite(
            row[f"ee_angular_velocity_{axis}"],
            f"ee_angular_velocity_{axis}",
        )
        for axis in "xyz"
    ]

    mep = [
        _finite(
            row[f"gt_mep_angular_velocity_{axis}"],
            f"gt_mep_angular_velocity_{axis}",
        )
        for axis in "xyz"
    ]

    return math.sqrt(
        sum(
            (a - b) ** 2
            for a, b in zip(ee, mep)
        )
    )


def capture_metrics_from_row(
    row,
    target_gap_m=DEFAULT_CAPTURE_TARGET_GAP_M,
):
    target_gap_m = _finite(
        target_gap_m,
        "target_gap_m",
    )

    gap = _finite(
        row["ee_gap_m"],
        "ee_gap_m",
    )

    lateral = abs(
        _finite(
            row["ee_lateral_error_m"],
            "ee_lateral_error_m",
        )
    )

    axial_error = gap - target_gap_m

    position_error = math.hypot(
        axial_error,
        lateral,
    )

    orientation_error = _first_finite(
        row,
        [
            "gt_ee_orientation_error_deg",
            "ee_normal_angle_deg",
        ],
    )

    total_velocity = _finite(
        row["gt_relative_velocity_mps"],
        "gt_relative_velocity_mps",
    )

    total_angular_velocity = math.degrees(
        _relative_angular_velocity_rad_s(row)
    )

    capture_success = _bool_value(
        row.get("capture_success", "0")
    )

    remaining_distance = (
        0.0
        if capture_success
        else max(0.0, axial_error)
    )

    return {
        "timestamp": str(
            row.get("timestamp", "")
        ),
        "state": str(
            row.get(
                "capture_state",
                "UNKNOWN",
            )
        ),
        "position_error": position_error,
        "orientation_error": orientation_error,
        "total_velocity": total_velocity,
        "total_angular_velocity": (
            total_angular_velocity
        ),
        "remaining_distance": (
            remaining_distance
        ),
        "capture_success": capture_success,
    }


class IsaacMetricsBridge(Node):

    def __init__(self):
        super().__init__(
            "isaac_metrics_bridge"
        )

        self.csv_path = Path(
            str(
                self.declare_parameter(
                    "csv_path",
                    DEFAULT_CSV_PATH,
                ).value
            )
        ).expanduser()

        self.poll_sec = float(
            self.declare_parameter(
                "poll_sec",
                0.1,
            ).value
        )

        self.target_gap_m = float(
            self.declare_parameter(
                "capture_target_gap_m",
                DEFAULT_CAPTURE_TARGET_GAP_M,
            ).value
        )

        base = "/mep_monitor/actual"

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
            f"{base}/relative_velocity",
            10,
        )

        self.pub_angular_velocity = (
            self.create_publisher(
                Float64,
                f"{base}/relative_angular_velocity",
                10,
            )
        )

        self.pub_remaining = self.create_publisher(
            Float64,
            f"{base}/remaining_distance",
            10,
        )

        self.pub_state = self.create_publisher(
            String,
            f"{base}/capture_state",
            10,
        )

        self.pub_success = self.create_publisher(
            Bool,
            f"{base}/capture_success",
            10,
        )

        self.pub_valid = self.create_publisher(
            Bool,
            f"{base}/metrics_valid",
            10,
        )

        self.last_timestamp = None
        self.last_state = None
        self.last_warning = None

        self.create_timer(
            self.poll_sec,
            self.publish_latest,
        )

        self.get_logger().info(
            f"MEP capture bridge ready | "
            f"CSV={self.csv_path} | "
            f"target_gap={self.target_gap_m:.3f} m"
        )

    def _reject(self, reason):
        self.pub_valid.publish(
            Bool(data=False)
        )

        if reason != self.last_warning:
            self.last_warning = reason
            self.get_logger().warning(reason)

    def _latest_row(self):
        if not self.csv_path.is_file():
            self._reject(
                f"Capture CSV not found: "
                f"{self.csv_path}"
            )
            return None

        latest = None

        with self.csv_path.open(
            "r",
            newline="",
            encoding="utf-8",
        ) as f:
            for row in csv.DictReader(f):
                latest = row

        return latest

    def publish_latest(self):
        try:
            row = self._latest_row()

            if row is None:
                return

            timestamp = str(
                row.get(
                    "timestamp",
                    "",
                )
            )

            if timestamp == self.last_timestamp:
                return

            metrics = capture_metrics_from_row(
                row,
                self.target_gap_m,
            )

            self.pub_position.publish(
                Float64(
                    data=metrics[
                        "position_error"
                    ]
                )
            )

            self.pub_orientation.publish(
                Float64(
                    data=metrics[
                        "orientation_error"
                    ]
                )
            )

            self.pub_velocity.publish(
                Float64(
                    data=metrics[
                        "total_velocity"
                    ]
                )
            )

            self.pub_angular_velocity.publish(
                Float64(
                    data=metrics[
                        "total_angular_velocity"
                    ]
                )
            )

            self.pub_remaining.publish(
                Float64(
                    data=metrics[
                        "remaining_distance"
                    ]
                )
            )

            self.pub_state.publish(
                String(
                    data=metrics["state"]
                )
            )

            self.pub_success.publish(
                Bool(
                    data=metrics[
                        "capture_success"
                    ]
                )
            )

            self.pub_valid.publish(
                Bool(data=True)
            )

            self.last_timestamp = timestamp
            self.last_warning = None

            if (
                metrics["state"]
                != self.last_state
            ):
                self.get_logger().info(
                    f'STATE={metrics["state"]} | '
                    f'POS={metrics["position_error"]:.4f} m | '
                    f'VEL={metrics["total_velocity"]:.4f} m/s | '
                    f'ANG={metrics["total_angular_velocity"]:.3f} deg/s | '
                    f'REMAIN={metrics["remaining_distance"]:.4f} m'
                )

                self.last_state = (
                    metrics["state"]
                )

        except Exception as exc:
            self._reject(
                f"Capture metrics rejected: "
                f"{exc}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = IsaacMetricsBridge()

    try:
        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        rclpy.executors.ExternalShutdownException,
    ):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
