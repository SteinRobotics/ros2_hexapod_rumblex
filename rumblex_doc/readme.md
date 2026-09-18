# Hardware
## PS3 controller
https://github.com/Ar-Ray-code/ps_ros2_common

old:
http://wiki.ros.org/ps3joy

## Servo
https://www.hiwonder.com/collections/bus-servo/products/lx-35h?variant=39700538982487

## Wire
connector 5264-3P

## Akku
Reely Modellbau-Akkupack (LiPo) 11.1 V 3700 mAh Zellen-Zahl: 3 40 C Hardcase XT90
(L x B x H) 139 x 46 x 25 mm

Zeee 3S Lipo Akku 50C 2200mAh 11,1V Kurze RC Batterie mit XT60 Stecker
75 x 34 x 26,5 mm (L x B x H); Gewicht ca.: 137 g
Stecker XT60
Ladestecker JST-XH

HOOVO 3S Lipo Akku 2200mAh 11,1V 50C SoftCase Kurze Batterie Lipo Battery mit XT60
75 x 34 x 26,5 mm
Stecker XT60
Ladestecker JST-XH

OVONIC 3S Lipo Akku 35C 2200mAh 11.1V Akku Kurzpack mit XT60
75 x 34 x 26,5 mm
Stecker XT60
Ladestecker JST-XH

## Sensors
### GY-BNO080
https://github.com/GAVLab/ros_bno08x

https://github.com/fm4dd/pi-bno080

from adafruit
https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/
https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/python-circuitpython
https://github.com/adafruit/Adafruit_CircuitPython_BNO08x

https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/

https://github.com/williamg42/BNO080-Linux-Library

### CJMCU-055 BNO055
https://www.davidpilling.com/wiki/index.php/BNO055?setskin=blog

https://github.com/adafruit/Adafruit_CircuitPython_BNO055

https://github.com/process1183/ros2_bno055      # python
https://github.com/bdholt1/ros2_bno055_sensor   # c++

pip3 install adafruit-circuitpython-bno055

There are two solder pads for PS0 and PS1 which have to be bridged to either + or - pads to set the operating mode - I2C, UART serial etc. For I2C both are bridged to -

The ATX pin is SDA, and the LRX pin SCL.
I2C pin set to ground.

-> address: 28
```
import adafruit_bno055
import board

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)
print(sensor.temperature)
print(sensor.euler)
print(sensor.gravity)
```

### LCD display 128x64
This has a GND, VCC, SCL & SDA pin which are then simply connected to the RaspberryPi.
ground to PIN6
VCC to PIN1,
SCL to PIN5
SDA to PIN3
But you can freely choose GND and VCC, it doesn't matter whether it's 3.3V or 5V.

-> address: 3c

pip3 install adafruit-circuitpython-ssd1306 --break-system-packages

https://github.com/adafruit/Adafruit_CircuitPython_SSD1306/blob/main/examples/ssd1306_pillow_text.py

(env) pi@raspberrypi:~ $ sudo wget -O font5x8.bin https://github.com/adafruit/Adafruit_CircuitPython_framebuf/blob/main/examples/font5x8.bin?raw=true

```
from board import SCL, SDA
import busio
import adafruit_ssd1306

i2c = busio.I2C(SCL, SDA)
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)
#display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, addr=0x31)

# Clear the display.  Always call show after changing pixels to make the display update visible!
display.fill(0)
display.show()

# Set a pixel in the origin 0,0 position.
display.pixel(0, 0, 1)
# Set a pixel in the middle 64, 16 position.
display.pixel(64, 16, 1)
# Set a pixel in the opposite 127, 31 position.
display.pixel(127, 31, 1)
display.show()

display.text("hi", 0, 1, True)
display.show()
```


### Garmin Lidar Lite v3
https://github.com/garmin/LIDARLite_RaspberryPi_Library/

pins
```
* LLv3 Blue  (SDA) - RPi pin 3 (GPIO 2)
* LLv3 Green (SCL) - RPi pin 5 (GPIO 3)
* LLv3 Red   (5V ) - RPi pin 4
* LLv3 Black (GND) - RPi pin 6

- Wire a 680uF capacitor across pins 4 and 6
```

### Loudspeaker
usb 4-1: new full-speed USB device number 2 using xhci-hcd
usb 4-1: device descriptor read/64, error -71
usb 4-1: New USB device found, idVendor=4c4a, idProduct=4155, bcdDevice= 1.00
usb 4-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
usb 4-1: Product: UACDemoV1.0
usb 4-1: Manufacturer: Jieli Technology
usb 4-1: SerialNumber: 503581119377601F
input: Jieli Technology UACDemoV1.0 as /devices/platform/axi/1000120000.pcie/1f00300000.usb/xhci-hcd.1/usb4/4-1/4-1:1.2/0003:4C4A:4155.0003/input/input9
hid-generic 0003:4C4A:4155.0003: input,hidraw2: USB HID v1.00 Device [Jieli Technology UACDemoV1.0] on usb-xhci-hcd.1-1/input2
usbcore: registered new interface driver snd-usb-audio

sudo apt-get install pulseaudio



