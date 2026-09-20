"""STEP 2 direct mock metrics publisher; never connect to controller inputs."""
import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64, Int32, String

from mep_monitor.mock_scenario import Scenario


class MockTelemetry(Node):
    def __init__(self):
        super().__init__('mock_telemetry', namespace='/mep_monitor/mock')
        # Reject accidental launch into a production namespace.
        if self.get_namespace() != '/mep_monitor/mock':
            raise ValueError('Mock node must remain in /mep_monitor/mock')
        self.declare_parameter('rate_hz', 20.0)
        defaults = Scenario()
        durations = {}
        for name in ('capture_seconds', 'attached_hold_seconds', 'docking_seconds', 'event_seconds'):
            durations[name] = self.declare_parameter(name, getattr(defaults, name)).value
        self.scenario = Scenario(**durations)
        rate = self.get_parameter('rate_hz').value
        if not math.isfinite(rate) or not 1.0 <= rate <= 200.0:
            raise ValueError('rate_hz must be finite and in [1, 200]')
        if self.scenario.event_seconds < 2.0 / rate:
            raise ValueError('event_seconds must span at least two publish periods')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.outputs = {}
        for name, value in self.scenario.sample(0.0).items():
            msg_type = {bool: Bool, int: Int32, float: Float64, str: String}[type(value)]
            self.outputs[name] = (self.create_publisher(msg_type, name, qos), msg_type)
        # Standalone mock must work without Isaac /clock, including use_sim_time=true.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.started = self.steady_clock.now()
        self.last_state = None
        self.timer = self.create_timer(1.0 / rate, self.publish_sample, clock=self.steady_clock)
        self.get_logger().warning('MOCK ONLY: scripted time events, no physics or capture/docking thresholds')

    def publish_sample(self):
        elapsed = (self.steady_clock.now() - self.started).nanoseconds / 1e9
        sample = self.scenario.sample(elapsed)
        for name, value in sample.items():
            publisher, msg_type = self.outputs[name]
            publisher.publish(msg_type(data=value))
        if sample['mission_state'] != self.last_state:
            self.last_state = sample['mission_state']
            self.get_logger().info(f'MOCK ONLY: {sample["mission_phase"]} / {self.last_state}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockTelemetry()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
