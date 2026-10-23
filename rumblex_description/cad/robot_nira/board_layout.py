"""PCB placements and mounting patterns shared by the base and assembly."""
from dataclasses import dataclass
from typing import Callable
from types import ModuleType

import boards.board_rpi5 as board_rpi5
import boards.board_servo_plug as board_servo_plug
import boards.board_relay as board_relay
import boards.board_ina228 as board_ina228
import boards.board_bno055 as board_bno055
import boards.board_servo_interface as board_servo_interface
import boards.board_i2c_distributor as board_i2c_distributor

@dataclass(frozen=True)
class Placement:
    """2D position plus rotation (around Z / the vertical axis) in the sketch plane."""

    x: float
    y: float
    rotation: float = 0.0


LOCATION_RPI5 = Placement(-34.0, -11.5)  # upper-left hole shares BNO055's (-5, 8) center
LOCATION_SERVO_PLUG = Placement(10.0, 32.5)  # combined ten-plug board; clears relay
LOCATION_RELAY = Placement(51.0, 32.0)  # links oben
LOCATION_BNO055 = Placement(0.0, -2.0, 90.0)  # mitte, ggf leicht verschoben
LOCATION_INA228 = Placement(-60.0, 33.0, 90.0)  # lower left, rotated 45 degrees clockwise
LOCATION_SERVO_INTERFACE = Placement(52.0, 0.0)  # oben, mitte
LOCATION_I2C_DISTRIBUTOR = Placement(-29.0, 35.0)  # lower left, beside Raspberry Pi


@dataclass(frozen=True)
class BoardMount:
    module: ModuleType
    placement: Placement
    hole_pattern: Callable
    name: str

    def holes(self):
        return self.hole_pattern(self.module.cfg)


BOARD_MOUNTS = (
    BoardMount(board_rpi5, LOCATION_RPI5, board_rpi5.default_hole_positions, "Raspberry Pi 5"),
    BoardMount(board_servo_plug, LOCATION_SERVO_PLUG, board_servo_plug.default_hole_positions, "Servo plugs (10)"),
    BoardMount(board_relay, LOCATION_RELAY, board_relay.default_hole_positions, "Relay"),
    BoardMount(board_bno055, LOCATION_BNO055, board_bno055.two_holes_on_one_side, "BNO055"),
    BoardMount(board_ina228, LOCATION_INA228, board_ina228.two_holes_on_one_side, "INA228"),
    BoardMount(board_servo_interface, LOCATION_SERVO_INTERFACE, board_servo_interface.default_hole_positions, "Servo interface"),
    BoardMount(board_i2c_distributor, LOCATION_I2C_DISTRIBUTOR, board_i2c_distributor.default_hole_positions, "I2C distributor"),
)
