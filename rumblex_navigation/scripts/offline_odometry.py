#!/usr/bin/env python3
"""Estimate offline motion from the commands used by the movement node.

Speech, joystick and cmd_vel inputs converge on cmd_movement_update in the brain.
Integrate only while the movement node reports a walking or running gait.
This node must not run alongside another odom -> base_link publisher.
"""

import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rumblex_interfaces.msg import ContinuousMovementUpdate, MovementRequest
from tf2_ros import TransformBroadcaster

from planar_motion import integrate_pose


class OfflineOdometry(Node):
    def __init__(self):
        super().__init__('offline_odometry')
        self.pose = (0.0, 0.0, 0.0)
        self.velocity = Twist()
        self.moving = False
        self.last_update = self.get_clock().now()
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            ContinuousMovementUpdate, 'cmd_movement_update', self.on_movement_update, 10)
        self.gait_subscription = self.create_subscription(
            MovementRequest, 'movement_type_actual', self.on_gait, 10)
        self.timer = self.create_timer(0.02, self.publish_pose)

    def advance(self, now):
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        if self.moving:
            self.pose = integrate_pose(*self.pose, self.velocity.linear.x,
                                       self.velocity.linear.y, self.velocity.angular.z, dt)

    def on_gait(self, gait):
        self.advance(self.get_clock().now())
        self.moving = gait.type in (MovementRequest.CONTINUOUS_MOVE,
                                   MovementRequest.CONTINUOUS_RUNNING)

    def on_movement_update(self, update):
        command = update.velocity
        if not all(math.isfinite(v) for v in
                   (command.linear.x, command.linear.y, command.angular.z)):
            self.get_logger().warning('Ignoring non-finite movement velocity')
            return
        # Account for the old velocity up to receipt of the new command.
        self.advance(self.get_clock().now())
        self.velocity = Twist()
        self.velocity.linear.x = command.linear.x
        self.velocity.linear.y = command.linear.y
        self.velocity.angular.z = command.angular.z

    def publish_pose(self):
        now = self.get_clock().now()
        self.advance(now)
        x, y, yaw = self.pose
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(transform)

        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = transform.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist = self.velocity if self.moving else Twist()
        self.odom.publish(odom)


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
