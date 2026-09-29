#!/usr/bin/env python3
"""Display extruded map walls and simulate a single-beam lidar for offline tests."""

import math

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Range
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from wall_geometry import ray_distance, rotate, wall_boxes


def quaternion_tuple(q):
    return q.x, q.y, q.z, q.w


class TestMapWalls(Node):
    def __init__(self):
        super().__init__('test_map_walls')
        self.height = self.declare_parameter('wall_height', 2.0).value
        if not math.isfinite(self.height) or self.height <= 0:
            raise ValueError('wall_height must be finite and positive')
        self.lidar_frame = self.declare_parameter('lidar_frame', 'lidar_link').value
        self.map = None
        self.boxes = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, 'test_map_walls', latched)
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.declare_parameter('simulate_lidar', True).value:
            self.ranges = self.create_publisher(Range, 'scan_1d', 10)
            self.timer = self.create_timer(0.05, self.publish_range)

    def on_map(self, grid):
        self.map = grid
        self.boxes = wall_boxes(grid.data, grid.info.width, grid.info.height,
                                grid.info.resolution, self.height)
        clear = Marker(action=Marker.DELETEALL)
        markers = MarkerArray(markers=[clear])
        pose = grid.info.origin
        for index, (lower, upper) in enumerate(self.boxes):
            marker = Marker()
            marker.header.frame_id = grid.header.frame_id
            marker.ns = 'walls'
            marker.id = index
            marker.type = Marker.CUBE
            center = rotate(tuple((a + b) / 2 for a, b in zip(lower, upper)),
                            quaternion_tuple(pose.orientation))
            marker.pose.position = Point(x=pose.position.x + center[0],
                                         y=pose.position.y + center[1],
                                         z=pose.position.z + center[2])
            marker.pose.orientation = pose.orientation
            marker.scale.x, marker.scale.y, marker.scale.z = (
                b - a for a, b in zip(lower, upper))
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.65, 0.7, 0.8, 0.8
            markers.markers.append(marker)
        self.markers.publish(markers)

    def publish_range(self):
        if self.map is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map.header.frame_id, self.lidar_frame, rclpy.time.Time())
        except TransformException:
            return
        translation = transform.transform.translation
        pose = self.map.info.origin
        qx, qy, qz, qw = quaternion_tuple(pose.orientation)
        inverse = (-qx, -qy, -qz, qw)
        origin = rotate((translation.x - pose.position.x, translation.y - pose.position.y,
                         translation.z - pose.position.z), inverse)
        direction = rotate(rotate((1.0, 0.0, 0.0),
                                  quaternion_tuple(transform.transform.rotation)), inverse)
        reading = Range()
        reading.header.frame_id = self.lidar_frame
        reading.header.stamp = transform.header.stamp
        reading.radiation_type = Range.INFRARED
        reading.field_of_view = 0.01
        reading.min_range = 0.05
        reading.max_range = 40.0
        distance = ray_distance(origin, direction, self.boxes, reading.max_range)
        reading.range = -math.inf if distance < reading.min_range else distance
        self.ranges.publish(reading)


def main():
    rclpy.init()
    node = TestMapWalls()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
