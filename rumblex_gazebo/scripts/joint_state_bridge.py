#!/usr/bin/env python3
"""Bridge between movement node's JointState output and Gazebo's ForwardPositionController.

Subscribes to 'target_joint_states' (sensor_msgs/JointState) published by the
movement node (remapped from 'joint_states') and republishes the positions as
Float64MultiArray on 'forward_position_controller/commands' in the joint order
expected by the controller config.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        self.declare_parameter('joint_names', rclpy.Parameter.Type.STRING_ARRAY)
        self.joint_names_ = self.get_parameter('joint_names').get_parameter_value().string_array_value
        if not self.joint_names_:
            raise RuntimeError('Parameter "joint_names" must not be empty')
        self.sub_ = self.create_subscription(
            JointState, 'target_joint_states', self.on_joint_states, 10
        )
        self.pub_ = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )

    def on_joint_states(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        out = Float64MultiArray()
        out.data = [name_to_pos.get(j, 0.0) for j in self.joint_names_]
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
