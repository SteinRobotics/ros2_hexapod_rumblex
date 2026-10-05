#!/usr/bin/env python3
"""Draw room names as floor-level labels over the house occupancy map."""

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


# Map-frame coordinates, measured from the current house floor plan.
ROOMS = (
    ('Dining', -2.26, 2.44),
    ('Kitchen', 2.58, 2.92),
    ('WC', 0.32, 3.49),
    ('Hallway', 0.35, 1.75),
    ('Living', -2.26, -2.13),
    ('TV', 2.11, -2.81),
)


class RoomLabels(Node):
    def __init__(self):
        super().__init__('room_labels')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, 'map_rooms', qos)
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.on_map, qos)

    def on_map(self, grid):
        labels = MarkerArray()
        for marker_id, (name, x, y) in enumerate(ROOMS):
            marker = Marker()
            marker.header.frame_id = grid.header.frame_id
            marker.ns = 'room_labels'
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.025
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.24
            marker.color.r = 0.08
            marker.color.g = 0.08
            marker.color.b = 0.08
            marker.color.a = 1.0
            marker.text = name
            labels.markers.append(marker)
        self.publisher.publish(labels)


def main():
    rclpy.init()
    node = RoomLabels()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
