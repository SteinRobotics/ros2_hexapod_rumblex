#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import subprocess
import time
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu, Range
from rumblex_interfaces.msg import MovementRequest
from rumblex_interfaces.msg import ServoStatus
from rumblex_interfaces.msg import JoystickRequest

from PIL import Image, ImageDraw, ImageFont

import board
from digitalio import DigitalInOut
import adafruit_ssd1306
import adafruit_ina228


class NodeHmi(Node):
    NUMBER_OF_LINES = 6

    def __init__(self):
        super().__init__('node_hmi')
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

        self.display_old_values = ["" for x in range(self.NUMBER_OF_LINES)]  # store the last texts to be able to overwrite them with black before writing new text

        # Servo Relay
        self.relay_pin = DigitalInOut(board.D16) # GPIO_16, physical pin 36  
        self.relay_pin.switch_to_output()
        self.relay_pin.value = False # turn relay off

        self.i2c = board.I2C()
        self.display = adafruit_ssd1306.SSD1306_I2C(128, 64, self.i2c)
        self.display.fill(0)
        self.display.show()
        self.image = Image.new("1", (self.display.width, self.display.height))
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", 12)

        self.__ina228 = adafruit_ina228.INA228(self.i2c)
        time.sleep(0.5)

        # turn relay on
        self.relay_pin.value = True

        self.create_subscription(Range, 'scan_1d', self.callback_lidar, 10)
        self.create_subscription(Bool, 'request_servo_relay', self.callback_servo_relay, 10)
        self.create_subscription(Bool, 'request_system_shutdown', self.callback_system_shutdown, 10)
        self.create_subscription(MovementRequest, 'movement_type_actual', self.callback_movement_type, 10)
        self.create_subscription(ServoStatus, 'servo_status', self.callback_servo_status, 10)
        self.create_subscription(JoystickRequest, 'joystick_request', self.callback_joystick_request, 10)
        self.create_subscription(Imu, 'bno055/imu', self.callback_imu, 10)
        self.pub_supply_voltage = self.create_publisher(Float32, 'supply_voltage', 10)

        # TODO TEST easier like this self.font_height = 10 + 2
        (self.font_width, self.font_height) = self.font.getmask("SYSTEM").size
        self.font_height += 3
        self.update_display("SYSTEM", 0)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def update_display(self, text, line_number=0):
        # first overwrite the old text with the same text in black
        self.draw.text((0, self.font_height * line_number), self.display_old_values[line_number], font=self.font, fill=0)
        # then write the new text in white
        self.draw.text((0, self.font_height * line_number), text, font=self.font, fill=255)
        self.display.image(self.image)
        self.display.show()
        # update the text 
        self.display_old_values[line_number] = text

    def get_ip_address(self):
        ip_address = "0.0.0.0"
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8",80))
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
        for line_number, text in enumerate(page_data):
            self.update_display(text, line_number)

    def update_ip_address(self):
        if self.is_ip_address_identified:
            return
        
        if not self.is_ip_address_identified and self.timeout_identifying_ip_address > 0:
            self.timeout_identifying_ip_address -= 1
            ip_address = self.get_ip_address()
            self.text_ip_address  = "IP: " + ip_address

    def callback_imu(self, msg):
        gz = msg.linear_acceleration.z
        self.text_bno055 = f"g: {gz:.2f}m/s^2"

    def update_ina228(self):
        voltage = self.__ina228.bus_voltage
        current = self.__ina228.current
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
        self.update_ina228()
        self.update_active_page()

    def callback_lidar(self, msg):
        self.text_lidar = f"d: {msg.range:>5.2f}m"   # right justified, 5 characters wide, 2 decimal places

    def callback_movement_type(self, msg):
        self.text_movement_request = msg.name.lower()

    def callback_servo_status(self, msg):
        self.text_servo_name = f"{msg.servo_max_temperature.lower()}"
        self.text_servo_min_voltage = f"U: {msg.min_voltage:.1f}V" 
        self.text_servo_max_temperature = f"T: {msg.max_temperature:.1f}°C"

    def callback_servo_relay(self, msg):
        self.get_logger().info('callback_servo_relay: %s' % msg.data)
        self.text_servo_relay_status = "RELAY ON" if msg.data else "RELAY OFF"
        if msg.data:
            self.relay_pin.value = True
        else:
            self.relay_pin.value = False

    def callback_joystick_request(self, msg):
        if msg.button_select:
            self.draw.rectangle((0, 0, self.display.width, self.display.height), fill=0)
            self.display_old_values = ["" for _ in range(self.NUMBER_OF_LINES)]
            idx = self.page_orders.index(self.active_page_name)
            self.active_page_name = self.page_orders[(idx + 1) % len(self.page_orders)]
            self.update_active_page()
    
    def callback_system_shutdown(self, msg):
        self.get_logger().info('callback_system_shutdown: %s' % msg.data)
        if msg.data:
            self.shutdown_callback()
            time.sleep(1)
            self.get_logger().info('3')
            time.sleep(1)
            self.get_logger().info('2')
            time.sleep(1)
            self.get_logger().info('1')
            time.sleep(1)
            subprocess.call("sudo shutdown -h now", shell=True)  # sudo visudo ->  rumblex ALL=(ALL) NOPASSWD: /sbin/shutdown

    def shutdown_callback(self):
        self.get_logger().info("shutdown_callback")
        self.relay_pin.value = False
        self.text_movement_request = "!!SHUTDOWN!!"
        self.active_page_name = 'system'
        self.update_active_page()


def main(args=None):
    rclpy.init(args=args)
    node_hmi = NodeHmi()

    try:
        rclpy.spin(node_hmi)  # Spin the node to process callbacks
    except KeyboardInterrupt:
        node_hmi.get_logger().info("Node interrupted by user.")
    finally:
        node_hmi.shutdown_callback()
        node_hmi.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
