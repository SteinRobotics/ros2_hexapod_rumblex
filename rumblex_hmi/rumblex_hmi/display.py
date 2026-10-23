"""SSD1306 initialization with bounded I2C framebuffer transfers."""


def create_display(width, height, bus):
    # Keep hardware dependencies out of the simulated backend.
    import adafruit_ssd1306

    class ChunkedSSD1306(adafruit_ssd1306.SSD1306_I2C):
        def write_framebuf(self):
            # The Nox Pi 5 bus rejects full-frame and 128-byte page writes.
            # Horizontal addressing preserves the pixel cursor between transfers;
            # each transfer needs its own data control byte (0x40).
            with self.i2c_device:
                for offset in range(1, len(self.buffer), 32):
                    self.i2c_device.write(b'\x40' + self.buffer[offset:offset + 32])

    return ChunkedSSD1306(width, height, bus)
