#!/usr/bin/env python3
"""Estimate body motion from applied toe targets, assuming no foot slip."""

import math

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rumblex_interfaces.msg import BodyPose
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from offline_motion import (
    LEG_NAMES, LOCOMOTION, body_velocity, compose_torso, support_displacement,
)


class OfflineOdometry(Node):
    def __init__(self, **kwargs):
        super().__init__('offline_odometry', **kwargs)
        self.heights = [self.declare_parameter(
            f'toe_positions_standing.{name}.z', rclpy.Parameter.Type.DOUBLE).value
            for name in LEG_NAMES]
        self.tolerance = self.declare_parameter('support_tolerance_m', 0.000001).value
        if not all(math.isfinite(z) for z in self.heights) or not (
                math.isfinite(self.tolerance) and self.tolerance > 0):
            raise ValueError('Standing heights must be finite and support tolerance positive')
        self.pose = (0.0, 0.0, 0.0)
        self.torso = BodyPose().torso_pose
        self.velocity = Twist()
        self.movement = ''
        self.previous = None
        self.last_update = None
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.estimated_velocity = self.create_publisher(
            TwistStamped, 'movement_velocity_estimated', 10)
        self.tf = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            BodyPose, 'body_pose_actual', self.on_body_pose,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.movement_subscription = self.create_subscription(
            String, 'movement_name', self.on_movement_name,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.02, self.publish_pose)

    def on_movement_name(self, message):
        if message.data != self.movement:
            self.previous = None
            self.velocity = Twist()
        self.movement = message.data

    def on_body_pose(self, message):
        toes = [(p.x, p.y, p.z) for p in message.toe_positions]
        p, r = message.torso_pose.position, message.torso_pose.orientation
        if not all(math.isfinite(v) for v in
                   [p.x, p.y, p.z, r.roll, r.pitch, r.yaw,
                    message.head_pose.roll, message.head_pose.pitch, message.head_pose.yaw]
                   + [v for toe in toes for v in toe]):
            self.previous = None
            self.last_update = None
            self.velocity = Twist()
            self.get_logger().warning('Ignoring non-finite offline body pose')
            return
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9 if self.last_update else 0.0
        old_body = compose_torso(self.pose, self.torso)
        self.velocity = Twist()
        if self.movement in LOCOMOTION and self.previous is not None and 0 < dt <= 0.5:
            delta = support_displacement(self.previous, toes, self.heights, self.tolerance)
            if delta is not None:
                dx, dy, turn = delta
                x, y, yaw = self.pose
                c, s = math.cos(yaw), math.sin(yaw)
                self.pose = (x + c * dx - s * dy, y + s * dx + c * dy,
                             math.atan2(math.sin(yaw + turn), math.cos(yaw + turn)))
        if self.last_update is not None and 0 < dt <= 0.5:
            linear, angular = body_velocity(old_body, compose_torso(self.pose, message.torso_pose), dt)
            self.velocity.linear.x, self.velocity.linear.y, self.velocity.linear.z = linear
            self.velocity.angular.x, self.velocity.angular.y, self.velocity.angular.z = angular
        self.previous = toes
        self.last_update = now
        self.torso = message.torso_pose
        self.publish_pose()

    def publish_pose(self):
        now = self.get_clock().now()
        if self.last_update is None or not (0 <= (now - self.last_update).nanoseconds <= 500_000_000):
            self.velocity = Twist()
        position, quaternion = compose_torso(self.pose, self.torso)
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        translation = transform.transform.translation
        translation.x, translation.y, translation.z = position
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = quaternion
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = transform.child_frame_id
        odom_position = odom.pose.pose.position
        odom_position.x, odom_position.y, odom_position.z = position
        odom.pose.pose.orientation = rotation
        odom.twist.twist = self.velocity
        self.odom.publish(odom)
        estimated = TwistStamped()
        estimated.header.stamp = now.to_msg()
        estimated.header.frame_id = 'base_link'
        estimated.twist = self.velocity
        self.estimated_velocity.publish(estimated)


def main(args=None):
    rclpy.init(args=args)
    node = OfflineOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
