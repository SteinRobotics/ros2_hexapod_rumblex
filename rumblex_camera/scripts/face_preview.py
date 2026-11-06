#!/usr/bin/env python3
"""Optional desktop view; closing it does not stop capture or robot visualization."""
import os
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        return
    rclpy.init()
    node = Node('face_preview')
    bridge = CvBridge()
    latest = [None]
    node.create_subscription(CompressedImage, 'perception/face_debug/compressed', lambda image: latest.__setitem__(0, image),
                             qos_profile_sensor_data)
    try:
        cv2.namedWindow('Nira face tracking', cv2.WINDOW_NORMAL)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if latest[0] is not None:
                cv2.imshow('Nira face tracking', bridge.compressed_imgmsg_to_cv2(latest[0], 'bgr8'))
                latest[0] = None
            if cv2.waitKey(1) == 27 or cv2.getWindowProperty('Nira face tracking', cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
