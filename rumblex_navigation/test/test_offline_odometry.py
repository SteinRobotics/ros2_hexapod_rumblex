"""Verify supporting-foot odometry and composed torso TF without wall-clock timing."""

import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.time import Time
from rumblex_interfaces.msg import BodyPose
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from offline_motion import LEG_NAMES, support_displacement
from offline_odometry import OfflineOdometry


class OfflineOdometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        parameters = [rclpy.Parameter(f'toe_positions_standing.{name}.z', value=-0.05)
                      for name in LEG_NAMES]
        self.node = OfflineOdometry(parameter_overrides=parameters)
        self.now = Time(seconds=100)
        self.clock = Mock()
        self.clock.now.side_effect = lambda: self.now
        self.clock_patch = patch.object(self.node, 'get_clock', return_value=self.clock)
        self.clock_patch.start()
        self.node.odom = Mock()
        self.node.tf = Mock()
        self.node.on_movement_name(String(data='CONTINUOUS_MOVE'))

    def tearDown(self):
        self.clock_patch.stop()
        self.node.destroy_node()

    def sample(self, dx=0.0, dy=0.0, yaw=0.0, lift=0.0):
        message = BodyPose()
        c, s = math.cos(yaw), math.sin(yaw)
        for p, (x, y) in zip(message.toe_positions,
                             [(0.2, 0.16), (0.0, 0.22), (-0.2, 0.16),
                              (0.2, -0.16), (0.0, -0.22), (-0.2, -0.16)]):
            p.x, p.y = c * (x - dx) + s * (y - dy), -s * (x - dx) + c * (y - dy)
            p.z = -0.05 + lift
        return message

    def send(self, message, dt=0.1):
        self.now = Time(nanoseconds=self.now.nanoseconds + int(dt * 1e9))
        self.node.on_body_pose(message)
        return self.node.odom.publish.call_args.args[0]

    def test_forward_sideways_and_turn(self):
        self.send(self.sample())
        odom = self.send(self.sample(dx=0.015, dy=0.002, yaw=0.03))
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.015)
        self.assertAlmostEqual(odom.pose.pose.position.y, 0.002)
        self.assertAlmostEqual(self.node.pose[2], 0.03)
        self.assertAlmostEqual(odom.twist.twist.angular.z, 0.3)

    def test_repeated_sample_stops_instead_of_extrapolating(self):
        self.send(self.sample())
        self.send(self.sample(dx=0.01))
        odom = self.send(self.sample(dx=0.01))
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.01)
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0.0)

    def test_no_support_holds_pose_and_reanchors_on_landing(self):
        self.send(self.sample())
        odom = self.send(self.sample(dx=0.01, lift=0.02))
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.0)
        self.send(self.sample(dx=0.02))
        odom = self.send(self.sample(dx=0.03))
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.01)

    def test_invalid_pose_and_long_gap_reset_history(self):
        self.send(self.sample())
        bad = self.sample()
        bad.torso_pose.orientation.roll = float('nan')
        self.node.on_body_pose(bad)
        self.send(self.sample(dx=0.01))
        odom = self.send(self.sample(dx=0.03), dt=2.0)
        self.assertEqual(odom.pose.pose.position.x, 0.0)

    def test_stationary_wave_composes_torso_once(self):
        self.node.on_movement_name(String(data='SEQUENCE_TORSO_ROLL'))
        message = self.sample()
        message.torso_pose.position.z = 0.01
        message.torso_pose.orientation.roll = 12.0
        odom = self.send(message)
        self.assertAlmostEqual(odom.pose.pose.position.z, 0.01)
        self.assertAlmostEqual(odom.pose.pose.orientation.x, math.sin(math.radians(6)))
        self.assertEqual(self.node.pose, (0.0, 0.0, 0.0))
        transform = self.node.tf.sendTransform.call_args.args[0]
        self.assertEqual(transform.transform.rotation, odom.pose.pose.orientation)
        self.assertEqual(transform.header, odom.header)

    def test_torso_twist_matches_rotation_and_translation(self):
        self.node.on_movement_name(String(data='SEQUENCE_BODY_ROLL'))
        self.send(self.sample())
        message = self.sample()
        message.torso_pose.position.z = 0.01
        message.torso_pose.orientation.roll = 12.0
        odom = self.send(message)
        self.assertAlmostEqual(odom.twist.twist.angular.x, math.radians(12) / 0.1)
        self.assertAlmostEqual(odom.twist.twist.linear.z, 0.1 * math.cos(math.radians(12)))
        self.assertAlmostEqual(odom.twist.twist.linear.y, 0.1 * math.sin(math.radians(12)))

    def test_feedback_timeout_holds_pose_and_clears_twist(self):
        self.send(self.sample())
        self.send(self.sample(dx=0.01))
        self.now = Time(nanoseconds=self.now.nanoseconds + 600_000_000)
        self.node.publish_pose()
        odom = self.node.odom.publish.call_args.args[0]
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.01)
        self.assertEqual(odom.twist.twist.linear.x, 0.0)

    def test_movement_change_resets_displacement(self):
        self.send(self.sample())
        self.node.on_movement_name(String(data='CONTINUOUS_RUNNING'))
        odom = self.send(self.sample(dx=0.01))
        self.assertEqual(odom.pose.pose.position.x, 0.0)

    def test_support_contact_change_uses_only_shared_feet(self):
        a, b = self.sample(), self.sample(dx=0.01)
        a.toe_positions[0].z += 0.02
        b.toe_positions[1].z += 0.02
        b.toe_positions[0].x += 0.1
        self.send(a)
        odom = self.send(b)
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.01)

    def test_robot_profiles_supply_standing_heights(self):
        import yaml
        config = Path(__file__).resolve().parents[2] / 'rumblex_description' / 'config'
        for robot in ('nox', 'nira'):
            profile = config / robot / 'anatomy.yaml'
            node = OfflineOdometry(cli_args=['--ros-args', '--params-file', str(profile)],
                                   use_global_arguments=False)
            try:
                expected = yaml.safe_load(profile.read_text())['/**']['ros__parameters']
                self.assertEqual(node.heights, [expected['toe_positions_standing'][name]['z']
                                                for name in LEG_NAMES])
            finally:
                node.destroy_node()

    def test_distinct_support_required(self):
        points = [(0.0, 0.0, -0.05)] * 6
        self.assertIsNone(support_displacement(points, points, [-0.05] * 6, 0.001))


if __name__ == '__main__':
    unittest.main()
