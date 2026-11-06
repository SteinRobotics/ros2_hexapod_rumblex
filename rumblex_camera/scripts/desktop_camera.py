#!/usr/bin/env python3
"""Desktop-only V4L2 capture. OpenCV negotiates formats with ordinary laptop webcams."""
from pathlib import Path
import glob
import math
import time
import struct
import fcntl
from urllib.parse import urlparse, unquote
import yaml

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage


def capture_devices():
    """Only return V4L2 capture nodes, deduplicating persistent symlinks."""
    paths = sorted(glob.glob('/dev/v4l/by-id/*')) + sorted(glob.glob('/dev/v4l/by-path/*'))
    paths += sorted(glob.glob('/dev/video*'))
    seen = set()
    result = []
    for path in paths:
        resolved = str(Path(path).resolve())
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            with open(resolved, 'rb', buffering=0) as device:
                capability = bytearray(104)
                fcntl.ioctl(device, 0x80685600, capability)  # VIDIOC_QUERYCAP
            caps, device_caps = struct.unpack_from('II', capability, 84)
            if caps & 0x80000000:
                caps = device_caps
            if caps & 0x00000001:  # VIDEO_CAPTURE, excludes metadata nodes
                result.append(path)
        except OSError:
            continue
    return result


class DesktopCamera(Node):
    def __init__(self):
        super().__init__('node_desktop_camera')
        cv2.setNumThreads(1)
        self.declare_parameter('video_device', '')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 15.0)
        self.declare_parameter('camera_info_url', '')
        self.calibration = None
        url = self.get_parameter('camera_info_url').value
        if url:
            parsed = urlparse(url)
            if parsed.scheme != 'file' or parsed.netloc not in ('', 'localhost'):
                raise ValueError('Desktop camera_info_url must be a local file:/// calibration YAML')
            self.calibration = yaml.safe_load(Path(unquote(parsed.path)).read_text())
            for key, size in (('camera_matrix', 9), ('rectification_matrix', 9), ('projection_matrix', 12)):
                values = self.calibration[key]['data']
                if len(values) != size or not all(math.isfinite(float(v)) for v in values):
                    raise ValueError(f'Invalid calibration {key}')
        device = self.get_parameter('video_device').value
        if not device:
            devices = capture_devices()
            if not devices:
                raise RuntimeError('No accessible V4L2 camera; check device permissions or camera_device override')
            device = devices[0]
            if len(devices) > 1:
                self.get_logger().warning(f'Multiple cameras: {devices}; selecting {device}. Use camera_device to override.')
        if not Path(device).is_absolute():
            raise ValueError('camera_device must be an absolute device path')
        rate = float(self.get_parameter('framerate').value)
        width = int(self.get_parameter('image_width').value)
        height = int(self.get_parameter('image_height').value)
        if not math.isfinite(rate) or not 1 <= rate <= 60 or not 64 <= width <= 1920 or not 64 <= height <= 1080:
            raise ValueError('Invalid desktop capture dimensions/rate')
        self.capture = cv2.VideoCapture(str(Path(device).resolve()), cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f'Cannot open camera {device}; check permissions and other capture applications')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, rate)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_logger().info(f'Camera {device}: {self.capture.get(cv2.CAP_PROP_FRAME_WIDTH):g}x'
                               f'{self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT):g}, '
                               f'{self.capture.get(cv2.CAP_PROP_FPS):g} fps; desktop uncalibrated preview')
        self.frames = 0
        self.report_time = time.monotonic()
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.compressed = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().error('Camera capture stopped; tracking will time out. Reconnect and restart launch.')
            self.timer.cancel()
            self.capture.release()
            return
        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # Host receive time: OpenCV does not expose the V4L2 exposure timestamp here.
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'desktop_camera_optical_frame'
        if self.images.get_subscription_count():
            self.images.publish(message)
        if self.compressed.get_subscription_count():
            encoded = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            encoded.header = message.header
            self.compressed.publish(encoded)
        info = CameraInfo(header=message.header, width=message.width, height=message.height)
        calibration = self.calibration
        if calibration and calibration['image_width'] == info.width and calibration['image_height'] == info.height:
            info.k = [float(v) for v in calibration['camera_matrix']['data']]
            info.r = [float(v) for v in calibration['rectification_matrix']['data']]
            info.p = [float(v) for v in calibration['projection_matrix']['data']]
            info.d = [float(v) for v in calibration['distortion_coefficients']['data']]
            info.distortion_model = calibration['distortion_model']
        self.info.publish(info)  # K[0] remains 0 without matching measured calibration.
        self.frames += 1
        elapsed = time.monotonic() - self.report_time
        if elapsed >= 10.0:
            self.get_logger().info(f'Desktop capture: {self.frames / elapsed:.1f} frames/s')
            self.frames = 0
            self.report_time = time.monotonic()

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = DesktopCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if rclpy.ok():
            rclpy.logging.get_logger('node_desktop_camera').error(str(error))
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
