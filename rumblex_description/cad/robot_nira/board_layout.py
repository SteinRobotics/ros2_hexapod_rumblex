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


LOCATION_RPI5 = Placement(-35.0, -6.0)  # rechts unten
LOCATION_LEFT_SERVO_PLUG = Placement(0.0, 25.0)  # links mitte, sollen zusammengesetzt werden
LOCATION_RIGHT_SERVO_PLUG = Placement(0.0, 40.0)  # links mitte, sollen zusammengesetzt werden
LOCATION_RELAY = Placement(36.0, 32.0)  # links oben
LOCATION_BNO055 = Placement(0.0, -2.0, 90.0)  # mitte, ggf leicht verschoben
LOCATION_INA228 = Placement(-65.0, -6.0, 90.0)  # mitte unten
LOCATION_SERVO_INTERFACE = Placement(35.0, 0.0)  # oben, mitte
LOCATION_I2C_DISTRIBUTOR = Placement(35.0, -30.0)  # oben, rechts


@dataclass(frozen=True)
class BoardMount:
    module: ModuleType
    placement: Placement
    hole_pattern: Callable

    def holes(self):
        return self.hole_pattern(self.module.cfg)


BOARD_MOUNTS = (
    BoardMount(board_rpi5, LOCATION_RPI5, board_rpi5.default_hole_positions),
    BoardMount(board_servo_plug, LOCATION_LEFT_SERVO_PLUG, board_servo_plug.default_hole_positions),
    BoardMount(board_servo_plug, LOCATION_RIGHT_SERVO_PLUG, board_servo_plug.default_hole_positions),
    BoardMount(board_relay, LOCATION_RELAY, board_relay.default_hole_positions),
    BoardMount(board_bno055, LOCATION_BNO055, board_bno055.two_holes_on_one_side),
    BoardMount(board_ina228, LOCATION_INA228, board_ina228.two_holes_on_one_side),
    BoardMount(board_servo_interface, LOCATION_SERVO_INTERFACE, board_servo_interface.default_hole_positions),
    BoardMount(board_i2c_distributor, LOCATION_I2C_DISTRIBUTOR, board_i2c_distributor.default_hole_positions),
)
