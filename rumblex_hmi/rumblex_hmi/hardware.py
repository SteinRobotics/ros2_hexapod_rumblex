"""Raspberry Pi GPIO, SSD1306 OLED, INA228, and host shutdown operations."""

import subprocess
import time

from rumblex_hmi.oled import OledRenderer


class HardwareBackend:
    def __init__(self):
        # Import only when hardware is selected; simulation needs none of these drivers.
        import board
        from digitalio import DigitalInOut
        import adafruit_ssd1306
        import adafruit_ina228

        self.relay_pin = None
        self.i2c = None
        self.display = None
        try:
            self.relay_pin = DigitalInOut(board.D16)  # GPIO 16, physical pin 36
            self.relay_pin.switch_to_output(value=False)
            self.i2c = board.I2C()
            self.display = adafruit_ssd1306.SSD1306_I2C(*OledRenderer.SIZE, self.i2c)
            self.display.fill(0)
            self.display.show()
            self.power_monitor = adafruit_ina228.INA228(self.i2c)
            time.sleep(0.5)
            self.set_relay(True)
        except BaseException:
            self.close()
            raise

    def present(self, image):
        self.display.image(image)
        self.display.show()

    def read_power(self):
        return self.power_monitor.bus_voltage, self.power_monitor.current

    def set_relay(self, enabled):
        self.relay_pin.value = enabled

    def shutdown_host(self, logger):
        time.sleep(1)
        for number in ('3', '2', '1'):
            logger.info(number)
            time.sleep(1)
        # Raspberry Pi sudoers must allow /sbin/shutdown without a password.
        subprocess.call(['sudo', 'shutdown', '-h', 'now'])

    def close(self):
        # Always attempt both resources, even if switching the relay off fails.
        try:
            if self.relay_pin is not None:
                try:
                    self.set_relay(False)
                finally:
                    self.relay_pin.deinit()
                    self.relay_pin = None
        finally:
            if self.i2c is not None:
                self.i2c.deinit()
                self.i2c = None
