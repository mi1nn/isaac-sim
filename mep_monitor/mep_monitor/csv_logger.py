"""CSV logger for MEP monitoring inputs and derived metrics.

This logger uses the stable CPU-side monitoring interface.

It does NOT assume:
- Isaac Sim source topic names
- mission-state logic
- PnP / GT availability
- timestamp synchronization rules

Each source timestamp is recorded separately so synchronization can be
verified later when the actual Isaac Sim interface is connected.
"""

import csv
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from mep_monitor.metrics import (
    orientation_error_deg,
    position_error,
    relative_linear_velocity,
)


FIELDNAMES = [
    'sample_index',
    'logger_time_sec',
    'frame_id',

    'target_pose_time_sec',
    'actual_pose_time_sec',
    'target_velocity_time_sec',
    'actual_velocity_time_sec',

    'target_x',
    'target_y',
    'target_z',

    'actual_x',
    'actual_y',
    'actual_z',

    'target_qx',
    'target_qy',
    'target_qz',
    'target_qw',

    'actual_qx',
    'actual_qy',
    'actual_qz',
    'actual_qw',

    'target_vx',
    'target_vy',
    'target_vz',

    'actual_vx',
    'actual_vy',
    'actual_vz',

    'error_x',
    'error_y',
    'error_z',

    'active_position_error',
    'orientation_error_deg',
    'relative_velocity_mps',

    'metrics_valid',
]


def build_metric_values(
    target_xyz,
    actual_xyz,
    target_xyzw,
    actual_xyzw,
    target_velocity_xyz,
    actual_velocity_xyz,
):
    """Calculate the same derived values used by monitoring_node."""

    ex, ey, ez, distance = position_error(
        target_xyz,
        actual_xyz,
    )

    angle = orientation_error_deg(
        target_xyzw,
        actual_xyzw,
    )

    speed = relative_linear_velocity(
        target_velocity_xyz,
        actual_velocity_xyz,
    )

    return {
        'error_x': ex,
        'error_y': ey,
        'error_z': ez,
        'active_position_error': distance,
        'orientation_error_deg': angle,
        'relative_velocity_mps': speed,
        'metrics_valid': True,
    }


def stamp_to_sec(stamp):
    return (
        float(stamp.sec)
        + float(stamp.nanosec) / 1e9
    )


class CsvLogger(Node):

    REQUIRED_INPUTS = {
        'target_pose',
        'actual_pose',
        'target_velocity',
        'actual_velocity',
    }

    def __init__(self):
        super().__init__(
            'csv_logger',
            namespace='/mep_monitor',
        )

        default_output_dir = (
            Path.home()
            / 'MRV_MEP_Project'
            / 'monitor_ws'
            / 'logs'
        )

        output_dir = Path(
            self.declare_parameter(
                'output_dir',
                str(default_output_dir),
            ).value
        ).expanduser()

        output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        filename = (
            'mep_monitor_'
            + datetime.now().strftime(
                '%Y%m%d_%H%M%S'
            )
            + '.csv'
        )

        self.output_path = (
            output_dir / filename
        )

        self.csv_file = self.output_path.open(
            'w',
            newline='',
            encoding='utf-8',
        )

        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=FIELDNAMES,
        )

        self.writer.writeheader()
        self.csv_file.flush()

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest = {}
        self.updated = set()

        self.sample_index = 0
        self.last_reject_reason = None

        self.create_subscription(
            PoseStamped,
            'input/target_pose',
            lambda msg: self._store(
                'target_pose',
                msg,
            ),
            qos,
        )

        self.create_subscription(
            PoseStamped,
            'input/actual_pose',
            lambda msg: self._store(
                'actual_pose',
                msg,
            ),
            qos,
        )

        self.create_subscription(
            TwistStamped,
            'input/target_velocity',
            lambda msg: self._store(
                'target_velocity',
                msg,
            ),
            qos,
        )

        self.create_subscription(
            TwistStamped,
            'input/actual_velocity',
            lambda msg: self._store(
                'actual_velocity',
                msg,
            ),
            qos,
        )

        self.get_logger().info(
            f'CSV logger ready: '
            f'{self.output_path}'
        )

        self.get_logger().warning(
            'CSV rows are CPU-pipeline samples, '
            'not verified Isaac data. '
            'Timestamp synchronization and '
            'mission-state fields are not '
            'configured yet.'
        )

    def _store(self, key, msg):

        self.latest[key] = msg
        self.updated.add(key)

        if self.REQUIRED_INPUTS.issubset(
            self.updated
        ):
            self.updated.clear()
            self._write_sample()

    def _reject(self, reason):

        if reason != self.last_reject_reason:
            self.last_reject_reason = reason
            self.get_logger().warning(reason)

    def _write_sample(self):

        target_pose = (
            self.latest['target_pose']
        )

        actual_pose = (
            self.latest['actual_pose']
        )

        target_velocity = (
            self.latest['target_velocity']
        )

        actual_velocity = (
            self.latest['actual_velocity']
        )

        frame_ids = (
            target_pose.header.frame_id,
            actual_pose.header.frame_id,
            target_velocity.header.frame_id,
            actual_velocity.header.frame_id,
        )

        if any(
            not frame_id
            for frame_id in frame_ids
        ):
            self._reject(
                'CSV sample rejected: '
                'every input must provide '
                'a non-empty frame_id.'
            )
            return

        if len(set(frame_ids)) != 1:
            self._reject(
                'CSV sample rejected: '
                'all four inputs must use '
                'one common frame.'
            )
            return

        tp = target_pose.pose.position
        ap = actual_pose.pose.position

        tq = target_pose.pose.orientation
        aq = actual_pose.pose.orientation

        tv = target_velocity.twist.linear
        av = actual_velocity.twist.linear

        target_xyz = (
            tp.x,
            tp.y,
            tp.z,
        )

        actual_xyz = (
            ap.x,
            ap.y,
            ap.z,
        )

        target_xyzw = (
            tq.x,
            tq.y,
            tq.z,
            tq.w,
        )

        actual_xyzw = (
            aq.x,
            aq.y,
            aq.z,
            aq.w,
        )

        target_v = (
            tv.x,
            tv.y,
            tv.z,
        )

        actual_v = (
            av.x,
            av.y,
            av.z,
        )

        try:
            metrics = build_metric_values(
                target_xyz,
                actual_xyz,
                target_xyzw,
                actual_xyzw,
                target_v,
                actual_v,
            )

        except ValueError as exc:
            self._reject(
                f'CSV sample rejected: {exc}'
            )
            return

        logger_time_sec = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        row = {
            'sample_index':
                self.sample_index,

            'logger_time_sec':
                logger_time_sec,

            'frame_id':
                frame_ids[0],

            'target_pose_time_sec':
                stamp_to_sec(
                    target_pose.header.stamp
                ),

            'actual_pose_time_sec':
                stamp_to_sec(
                    actual_pose.header.stamp
                ),

            'target_velocity_time_sec':
                stamp_to_sec(
                    target_velocity.header.stamp
                ),

            'actual_velocity_time_sec':
                stamp_to_sec(
                    actual_velocity.header.stamp
                ),

            'target_x': tp.x,
            'target_y': tp.y,
            'target_z': tp.z,

            'actual_x': ap.x,
            'actual_y': ap.y,
            'actual_z': ap.z,

            'target_qx': tq.x,
            'target_qy': tq.y,
            'target_qz': tq.z,
            'target_qw': tq.w,

            'actual_qx': aq.x,
            'actual_qy': aq.y,
            'actual_qz': aq.z,
            'actual_qw': aq.w,

            'target_vx': tv.x,
            'target_vy': tv.y,
            'target_vz': tv.z,

            'actual_vx': av.x,
            'actual_vy': av.y,
            'actual_vz': av.z,
        }

        row.update(metrics)

        self.writer.writerow(row)
        self.csv_file.flush()

        self.sample_index += 1
        self.last_reject_reason = None

    def destroy_node(self):

        if (
            hasattr(self, 'csv_file')
            and not self.csv_file.closed
        ):
            self.csv_file.flush()
            self.csv_file.close()

        return super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = CsvLogger()
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


if __name__ == '__main__':
    main()
