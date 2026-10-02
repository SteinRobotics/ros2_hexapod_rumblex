# RumbleX HMI

The shared `NodeHmi` owns ROS subscriptions, page content, IP discovery, refreshes,
and supply-voltage publication. `OledRenderer` creates the same monochrome 128×64
Pillow frame for both backends. `HardwareBackend` owns the SSD1306, INA228, GPIO
relay, and host shutdown; `SimulatedBackend` presents frames in a Pygame window.

Build and source the workspace, then launch the desktop OLED:

```bash
ros2 launch rumblex_hmi hmi_simulated_launch.py
ros2 launch rumblex_hmi hmi_simulated_launch.py simulated_voltage:=11.8 simulated_current:=0.6
```

The window displays the OLED at 4× scale. A graphical desktop is required.
Simulation uses Pillow, Pygame, and the Liberation Sans font, declared in
`package.xml`; it requires no GPIO, I²C, or Adafruit libraries.

Cycle SYSTEM → SERVOS → SENSORS with `joystick_request.button_select`, either
from the physical controller or `ros2 launch rumblex_teleop teleop_simulated_launch.py`
(Backspace is Select). Telemetry topics and the one-second refresh interval are
identical to the hardware node, including `movement_name`, `servo_status`,
`scan_1d`, and `bno055/imu`.

Synthetic voltage/current default to 12.0 V / 0.0 A. These are simulator defaults,
not robot configuration. The node publishes `supply_voltage` every second;
current is displayed as an absolute value. Parameters can also change while running:

```bash
ros2 param set /node_hmi simulated_voltage 11.5
ros2 param set /node_hmi simulated_current 0.8
```

Relay requests update in-memory state. A true `request_system_shutdown` turns the
simulated relay off and shows `!!SHUTDOWN!!` on SYSTEM; the window and development
computer remain running. Closing the window or pressing Ctrl+C cleans up ROS and
the GUI. Run only one HMI backend in the same ROS namespace to avoid competing
`supply_voltage` publishers and duplicate node names.

The existing hardware entry point remains:

```bash
ros2 launch rumblex_hmi hmi_launch.py
```

Hardware additionally needs Adafruit Blinka (`board`/`digitalio`),
`adafruit-circuitpython-ssd1306`, and `adafruit-circuitpython-ina228`, enabled I²C,
and GPIO access on the Raspberry Pi. Host shutdown still requires the existing
sudoers permission for `/sbin/shutdown`. Hardware initialization keeps the relay
off until OLED and power-monitor setup completes, then turns it on.

Validate with:

```bash
colcon test --packages-select rumblex_hmi --event-handlers console_direct+
colcon test-result --verbose
```

GUI tests use SDL's dummy video driver and run without a desktop.
