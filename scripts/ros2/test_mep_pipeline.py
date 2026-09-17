"""CPU/ROS tests: source ROS first, then python3 -m unittest discover -s scripts/ros2."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.time import Time
from rclpy.clock import ClockType
from sensor_msgs.msg import Image

from grasp_geometry import backproject, grasp_from_points, pose_error
from mep_pose_controller import MepPoseController
from mep_rgbd_pose import image_array


class GeometryTest(unittest.TestCase):
    def test_metric_backprojection_and_invalid_depth(self):
        k = np.array([[100., 0, 1], [0, 100, 1], [0, 0, 1]])
        depth = np.ones((3, 3))*2
        depth[0, 0] = np.nan
        points = backproject(np.ones((3, 3)), depth, k, min_pixels=1)
        self.assertEqual(points.shape, (8, 3))
        np.testing.assert_allclose(points[-1], [.02, .02, 2])
        self.assertIsNone(backproject(np.zeros((3, 3)), depth, k))

    def test_grasp_axes_and_surface_compensation(self):
        points = np.column_stack((np.linspace(-.3, .3, 100), np.zeros(100), np.full(100, 2.)))
        center, q, axis = grasp_from_points(points, np.zeros(3), np.array([1., 0, 0]), .075)
        np.testing.assert_allclose(center, [0, 0, 2.075], atol=1e-8)
        np.testing.assert_allclose(axis, [1, 0, 0], atol=1e-8)
        from scipy.spatial.transform import Rotation
        np.testing.assert_allclose(-Rotation.from_quat(q).as_matrix()[:, 2], [0, 0, 1])

    def test_image_row_padding(self):
        msg = Image(height=2, width=1, encoding="rgb8", step=4,
                    data=bytes([1, 2, 3, 0, 4, 5, 6, 0]))
        np.testing.assert_array_equal(image_array(msg), [[[1, 2, 3]], [[4, 5, 6]]])

    def test_error_is_in_base_frame(self):
        from scipy.spatial.transform import Rotation
        current = Rotation.from_euler("z", 90, degrees=True)
        target = Rotation.from_euler("x", 10, degrees=True)*current
        dp, dr = pose_error(np.array([1., 0, 0]), target.as_quat(), np.zeros(3), current.as_quat())
        np.testing.assert_allclose(dp, [1, 0, 0])
        np.testing.assert_allclose(dr, [np.deg2rad(10), 0, 0], atol=1e-8)


class ControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MepPoseController()
        self.now = Time(seconds=10, clock_type=ClockType.ROS_TIME)
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: self.now)
        self.node.twist = SimpleNamespace(publish=Mock())
        self.node.gripper = SimpleNamespace(publish=Mock())
        self.node.status = SimpleNamespace(publish=Mock())
        tf = TransformStamped()
        tf.header.stamp = Time(seconds=10).to_msg()
        tf.transform.rotation.w = 1.
        self.node.tf = SimpleNamespace(lookup_transform=Mock(return_value=tf))

    def tearDown(self):
        self.node.destroy_node()

    def message(self, seconds=9.9, frame="mrv/base_link"):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = Time(seconds=seconds).to_msg()
        msg.pose.position.x = 1.
        msg.pose.orientation.w = 1.
        return msg

    def test_rejects_stale_future_wrong_frame_and_zero_quaternion(self):
        for msg in (self.message(9.), self.message(11.), self.message(frame="camera")):
            self.node.receive(msg)
            self.assertIsNone(self.node.pose)
        msg = self.message()
        msg.pose.orientation.w = 0.
        self.node.receive(msg)
        self.assertIsNone(self.node.pose)

    def test_waits_without_random_search(self):
        self.node.step()
        cmd = self.node.twist.publish.call_args.args[0]
        self.assertEqual(cmd.linear.x, 0.)
        self.assertEqual(cmd.angular.z, 0.)

    def test_pose_commands_then_stale_stops(self):
        self.node.receive(self.message())
        self.node.step()
        cmd = self.node.twist.publish.call_args.args[0]
        self.assertGreater(cmd.linear.x, 0.)
        self.assertLessEqual(np.linalg.norm([cmd.linear.x, cmd.linear.y, cmd.linear.z]), .030001)
        self.now = Time(seconds=10.45, clock_type=ClockType.ROS_TIME)
        self.node.step()
        cmd = self.node.twist.publish.call_args.args[0]
        self.assertEqual(cmd.linear.x, 0.)
        self.assertEqual(cmd.linear.z, 0.)

    def test_duplicate_does_not_refresh_watchdog(self):
        msg = self.message()
        self.node.receive(msg)
        self.node.received = 0.
        self.node.receive(msg)
        self.assertEqual(self.node.received, 0.)

    def test_close_requires_position_and_orientation(self):
        from scipy.spatial.transform import Rotation
        msg = self.message()
        msg.pose.position.x = 0.
        q = Rotation.from_euler("z", 45, degrees=True).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.node.state = "FINAL"
        self.node.receive(msg)
        self.node.step()
        self.assertEqual(self.node.state, "FINAL")
        self.assertFalse(self.node.gripper.publish.call_args.args[0].data)


if __name__ == "__main__":
    unittest.main()
