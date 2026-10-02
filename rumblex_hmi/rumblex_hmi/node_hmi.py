#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Imu, Range
from rumblex_interfaces.msg import ServoStatus
from rumblex_interfaces.msg import JoystickRequest

from rumblex_hmi.oled import HmiBackend, OledRenderer


class NodeHmi(Node):
    NUMBER_OF_LINES = 6

    def __init__(self, backend: HmiBackend | None = None):
        super().__init__('node_hmi')
        try:
            self._initialize(backend)
        except BaseException:
            try:
                if backend is None and hasattr(self, 'backend'):
                    self.backend.close()
            finally:
                self.destroy_node()
            raise

    def _initialize(self, backend):
        self.renderer = OledRenderer()
        if backend is None:
            from rumblex_hmi.hardware import HardwareBackend
            backend = HardwareBackend()
        self.backend = backend
        self.is_ip_address_identified = False
        self.timeout_identifying_ip_address = 60

        # System display text lines
        self.text_ip_address = ""  # line number 1
        self.text_movement_request = ""  # line number 2
        self.text_supply_voltage_and_current = ""  # line number 3
        self.text_cpu_temperature = ""  # line number 4

        # Servo text lines
        self.text_servo_relay_status = ""  # line number 1
        self.text_servo_name = ""  # line number 2
        self.text_servo_max_temperature = ""  # line number 3
        self.text_servo_min_voltage = ""  # line number 4

        # Sensor text lines
        self.text_listening_active = ""  # line number 1
        self.text_bno055 = ""  # line number 2
        self.text_lidar = ""  # line number 3

        self.active_page_name = 'system'
        self.page_orders = ['system', 'servos', 'sensors']

        self.create_subscription(Range, 'scan_1d', self.callback_lidar, 10)
        self.create_subscription(Bool, 'request_servo_relay', self.callback_servo_relay, 10)
        self.create_subscription(Bool, 'request_system_shutdown', self.callback_system_shutdown, 10)
        self.create_subscription(String, 'movement_name', self.callback_movement_type, 10)
        self.create_subscription(ServoStatus, 'servo_status', self.callback_servo_status, 10)
        self.create_subscription(JoystickRequest, 'joystick_request', self.callback_joystick_request, 10)
        self.create_subscription(Imu, 'bno055/imu', self.callback_imu, 10)
        self.pub_supply_voltage = self.create_publisher(Float32, 'supply_voltage', 10)

        self.update_active_page()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def get_ip_address(self):
        ip_address = "0.0.0.0"
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                ip_address = s.getsockname()[0]
            self.is_ip_address_identified = True
        except Exception as e:
            self.get_logger().warning('Could not get IP address, error: %s' % e)
            ip_address = "0.0.0.0"  # fallback

        self.get_logger().info('IP address: %s' % ip_address)
        return ip_address

    def get_page_data(self, page_name):
        if page_name == 'system':
            return [
                "SYSTEM",
                self.text_ip_address,
                self.text_movement_request,
                self.text_supply_voltage_and_current,
                self.text_cpu_temperature,
                ""
            ]
        elif page_name == 'servos':
            return [
                "SERVOS",
                self.text_servo_relay_status,
                self.text_servo_name,
                self.text_servo_max_temperature,
                self.text_servo_min_voltage,
                ""
            ]
        elif page_name == 'sensors':
            return [
                "SENSORS",
                self.text_listening_active,
                self.text_bno055,
                self.text_lidar,
                "",
                ""
            ]
        return [""] * self.NUMBER_OF_LINES

    def update_active_page(self):
        page_data = self.get_page_data(self.active_page_name)
        self.backend.present(self.renderer.render(page_data))

    def update_ip_address(self):
        if self.is_ip_address_identified:
            return

        if not self.is_ip_address_identified and self.timeout_identifying_ip_address > 0:
            self.timeout_identifying_ip_address -= 1
            ip_address = self.get_ip_address()
            self.text_ip_address = "IP: " + ip_address

    def callback_imu(self, msg):
        gz = msg.linear_acceleration.z
        self.text_bno055 = f"g: {gz:.2f}m/s^2"

    def update_power(self):
        voltage, current = self.backend.read_power()
        if voltage is None or current is None:
            return
        current = abs(current)
        self.text_supply_voltage_and_current = f"supply V: {voltage:.1f}V I: {current:.1f}A"
        # publish supply voltage
        msg = Float32()
        msg.data = voltage
        self.pub_supply_voltage.publish(msg)

    def timer_callback(self):
        self.update_ip_address()
        self.update_power()
        self.update_active_page()

    def callback_lidar(self, msg):
        self.text_lidar = f"d: {msg.range:>5.2f}m"   # right justified, 5 characters wide, 2 decimal places

    def callback_movement_type(self, msg):
        self.text_movement_request = msg.data.lower()

    def callback_servo_status(self, msg):
        self.text_servo_name = f"{msg.servo_max_temperature.lower()}"
        self.text_servo_min_voltage = f"U: {msg.min_voltage:.1f}V"
        self.text_servo_max_temperature = f"T: {msg.max_temperature:.1f}°C"

    def callback_servo_relay(self, msg):
        self.get_logger().info('callback_servo_relay: %s' % msg.data)
        self.text_servo_relay_status = "RELAY ON" if msg.data else "RELAY OFF"
        self.backend.set_relay(msg.data)

    def callback_joystick_request(self, msg):
        if msg.button_select:
            idx = self.page_orders.index(self.active_page_name)
            self.active_page_name = self.page_orders[(idx + 1) % len(self.page_orders)]
            self.update_active_page()

    def callback_system_shutdown(self, msg):
        self.get_logger().info('callback_system_shutdown: %s' % msg.data)
        if msg.data:
            self.shutdown_callback()
            self.backend.shutdown_host(self.get_logger())

    def shutdown_callback(self):
        self.get_logger().info("shutdown_callback")
        self.backend.set_relay(False)
        self.text_servo_relay_status = "RELAY OFF"
        self.text_movement_request = "!!SHUTDOWN!!"
        self.active_page_name = 'system'
        self.update_active_page()


def main(args=None):
    from rumblex_hmi.hardware import HardwareBackend

    rclpy.init(args=args)
    backend = None
    node_hmi = None
    try:
        backend = HardwareBackend()
        node_hmi = NodeHmi(backend)
        rclpy.spin(node_hmi)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node_hmi is not None:
                try:
                    node_hmi.shutdown_callback()
                finally:
                    node_hmi.destroy_node()
        finally:
            try:
                if backend is not None:
                    backend.close()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    main()
