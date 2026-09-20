"""ROS2 monitoring node for active target vs actual state metrics.

This node does not know Isaac Sim topic names, robot frames, or mission logic.
An upstream adapter must provide the currently active target and actual state
in one verified common coordinate frame.

Input quaternion convention follows geometry_msgs/Quaternion fields:
    x, y, z, w
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64

from mep_monitor.metrics import (
    orientation_error_deg,
    position_error,
    relative_linear_velocity,
)


class MonitoringNode(Node):

    REQUIRED_INPUTS = {
        'target_pose',
        'actual_pose',
        'target_velocity',
        'actual_velocity',
    }

    def __init__(self):
        super().__init__('monitoring_node', namespace='/mep_monitor')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest = {}
        self.updated = set()
        self.last_reject_reason = None

        # CPU-side stable monitoring interface.
        # Actual Isaac Sim topics will be mapped later after GPU-side verification.
        self.create_subscription(
            PoseStamped,
            'input/target_pose',
            lambda msg: self._store('target_pose', msg),
            qos,
        )

        self.create_subscription(
            PoseStamped,
            'input/actual_pose',
            lambda msg: self._store('actual_pose', msg),
            qos,
        )

        self.create_subscription(
            TwistStamped,
            'input/target_velocity',
            lambda msg: self._store('target_velocity', msg),
            qos,
        )

        self.create_subscription(
            TwistStamped,
            'input/actual_velocity',
            lambda msg: self._store('actual_velocity', msg),
            qos,
        )

        self.metric_publishers = {
            'active_position_error': self.create_publisher(
                Float64, 'active_position_error', qos
            ),
            'error_x': self.create_publisher(
                Float64, 'error_x', qos
            ),
            'error_y': self.create_publisher(
                Float64, 'error_y', qos
            ),
            'error_z': self.create_publisher(
                Float64, 'error_z', qos
            ),
            'orientation_error': self.create_publisher(
                Float64, 'orientation_error', qos
            ),
            'relative_velocity': self.create_publisher(
                Float64, 'relative_velocity', qos
            ),
            'metrics_valid': self.create_publisher(
                Bool, 'metrics_valid', qos
            ),
        }

        self.get_logger().info(
            'Monitoring node ready. '
            'Waiting for four /mep_monitor/input/* streams.'
        )

        self.get_logger().warning(
            'Isaac topic mapping, frame mapping, timestamp synchronization, '
            'and mission-phase target selection are NOT configured in this node.'
        )

    def _store(self, key, msg):
        self.latest[key] = msg
        self.updated.add(key)

        # One metric sample is produced only after all four inputs
        # have received a fresh update.
        if self.REQUIRED_INPUTS.issubset(self.updated):
            self.updated.clear()
            self._compute_and_publish()

    def _reject(self, reason):
        self.metric_publishers['metrics_valid'].publish(
            Bool(data=False)
        )

        if reason != self.last_reject_reason:
            self.last_reject_reason = reason
            self.get_logger().warning(reason)

    def _compute_and_publish(self):
        target_pose = self.latest['target_pose']
        actual_pose = self.latest['actual_pose']
        target_velocity = self.latest['target_velocity']
        actual_velocity = self.latest['actual_velocity']

        frame_ids = (
            target_pose.header.frame_id,
            actual_pose.header.frame_id,
            target_velocity.header.frame_id,
            actual_velocity.header.frame_id,
        )

        # Never calculate errors between unknown/different frames.
        if any(not frame_id for frame_id in frame_ids):
            self._reject(
                'Metrics rejected: every input must provide '
                'a non-empty frame_id.'
            )
            return

        if len(set(frame_ids)) != 1:
            self._reject(
                'Metrics rejected: target/actual pose and velocity '
                'must use one common frame.'
            )
            return

        tp = target_pose.pose.position
        ap = actual_pose.pose.position

        tq = target_pose.pose.orientation
        aq = actual_pose.pose.orientation

        tv = target_velocity.twist.linear
        av = actual_velocity.twist.linear

        try:
            ex, ey, ez, distance = position_error(
                (tp.x, tp.y, tp.z),
                (ap.x, ap.y, ap.z),
            )

            angle = orientation_error_deg(
                (tq.x, tq.y, tq.z, tq.w),
                (aq.x, aq.y, aq.z, aq.w),
            )

            speed = relative_linear_velocity(
                (tv.x, tv.y, tv.z),
                (av.x, av.y, av.z),
            )

        except ValueError as exc:
            self._reject(
                f'Metrics rejected: {exc}'
            )
            return

        values = {
            'active_position_error': distance,
            'error_x': ex,
            'error_y': ey,
            'error_z': ez,
            'orientation_error': angle,
            'relative_velocity': speed,
        }

        for name, value in values.items():
            self.metric_publishers[name].publish(
                Float64(data=value)
            )

        self.metric_publishers['metrics_valid'].publish(
            Bool(data=True)
        )

        self.last_reject_reason = None


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MonitoringNode()
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
