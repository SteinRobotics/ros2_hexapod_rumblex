"""Check movement-stream odometry without depending on wall-clock timing."""

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.time import Time
from rumblex_interfaces.msg import ContinuousMovementUpdate, MovementRequest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from offline_odometry import OfflineOdometry


class OfflineOdometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = OfflineOdometry()
        self.now = Time(seconds=100)
        self.node.last_update = self.now
        self.clock = Mock()
        self.clock.now.side_effect = lambda: self.now
        self.clock_patch = patch.object(self.node, 'get_clock', return_value=self.clock)
        self.clock_patch.start()
        self.node.odom = Mock()
        self.node.tf = Mock()

    def tearDown(self):
        self.clock_patch.stop()
        self.node.destroy_node()

    def update_velocity(self, x):
        update = ContinuousMovementUpdate()
        update.velocity.linear.x = x
        self.node.on_movement_update(update)

    def gait(self, gait_type):
        self.node.on_gait(MovementRequest(type=gait_type))

    def tick(self, seconds):
        self.now = Time(nanoseconds=self.now.nanoseconds + int(seconds * 1e9))
        self.node.publish_pose()
        return self.node.odom.publish.call_args.args[0]

    def test_speech_velocity_waits_for_standup_then_walks(self):
        self.update_velocity(0.005)
        self.gait(MovementRequest.SEQUENCE_STAND_UP)
        odom = self.tick(2)
        self.assertEqual(odom.pose.pose.position.x, 0.0)
        self.assertEqual(odom.twist.twist.linear.x, 0.0)
        self.gait(MovementRequest.CONTINUOUS_MOVE)
        odom = self.tick(10)
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.05)
        transform = self.node.tf.sendTransform.call_args.args[0]
        self.assertEqual(transform.header, odom.header)
        self.assertEqual(transform.child_frame_id, 'base_link')
        self.assertEqual(transform.transform.translation.x, odom.pose.pose.position.x)

    def test_zero_velocity_stops_walking(self):
        self.gait(MovementRequest.CONTINUOUS_MOVE)
        self.update_velocity(0.2)
        self.tick(1)
        self.update_velocity(0.0)
        odom = self.tick(5)
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.2)
        self.assertEqual(odom.twist.twist.linear.x, 0.0)

    def test_non_walking_gait_stops_even_with_retained_velocity(self):
        self.gait(MovementRequest.CONTINUOUS_RUNNING)
        self.update_velocity(0.2)
        self.tick(1)
        self.gait(MovementRequest.CONTINUOUS_POSE)
        odom = self.tick(5)
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.2)
        self.assertEqual(odom.twist.twist.linear.x, 0.0)

    def test_velocity_change_accounts_for_previous_interval(self):
        self.gait(MovementRequest.CONTINUOUS_MOVE)
        self.update_velocity(0.1)
        self.now = Time(seconds=102)
        self.update_velocity(0.2)
        odom = self.tick(1)
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.4)

    def test_invalid_velocity_does_not_corrupt_pose(self):
        self.gait(MovementRequest.CONTINUOUS_MOVE)
        self.update_velocity(0.1)
        self.update_velocity(float('nan'))
        odom = self.tick(1)
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.1)


if __name__ == '__main__':
    unittest.main()
