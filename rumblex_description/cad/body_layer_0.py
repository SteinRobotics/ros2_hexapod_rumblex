#!/usr/bin/env python3
"""Body layer 0: base plate with chamfered corner cutouts and PCB mounting holes."""

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

from build123d import *
from utils.ocp_utils import show

import body_common
import board_rpi5
import board_servo_plug
import board_relay
import board_ina228
import board_bno055
import board_servo_interface
import board_i2c_distributor

# orientation
#         ^ x
#         |
#         |
#    <----x
#    y
#
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


TOE_MOUNTING_HOLES = [
    (75.0, 37.0, 1.500),
    (75.0, -37.0, 1.500),
    (-75.0, -37.0, 1.500),
    (-75.0, 37.0, 1.500),
]

OCTAGON_POSITIONS = [
    body_common.octagon_position_left_top,
    body_common.octagon_position_right_top,
    body_common.octagon_position_right_bottom,
    body_common.octagon_position_left_bottom,
]

OUTPUT_DIR = Path("generated")
OUTPUT_NAME = "body_layer_0"


# TODO: move to generic helper file or to body_common.py
def chamfered_octagon(width: float, height: float, chamfer_length: float, rotation: float = 0.0) -> Sketch:
    with BuildSketch() as sk:
        Rectangle(width, height, rotation=rotation)
        chamfer(sk.vertices(), chamfer_length)
    return sk.sketch


def place_board_holes(
    hole_positions: Iterable[tuple[float, float]],
    placement: Placement,
    hole_radius: float,
) -> None:
    """Subtract a board's mounting holes into the current sketch.

    The hole pattern is rotated around the board's own center (local origin)
    first, and only then moved to the placement offset - so a nonzero
    `placement.rotation` spins the footprint in place instead of swinging it
    around the sketch origin.
    """
    offset = Location((placement.x, placement.y))
    rotation = Rotation(0, 0, placement.rotation)
    for x, y in hole_positions:
        with Locations(offset * rotation * Location((x, y))):
            Circle(hole_radius, mode=Mode.SUBTRACT)


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(body_common.base_plate)

        for pos in OCTAGON_POSITIONS:
            with Locations(pos):
                add(chamfered_octagon(55, 55, 4))

        for loc in body_common.hole_locations:
            with Locations(loc):
                Circle(body_common.hole_radius, mode=Mode.SUBTRACT)

        for x, y, radius in TOE_MOUNTING_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        # pcbs
        place_board_holes(
            board_rpi5.default_hole_positions(board_rpi5.cfg),
            LOCATION_RPI5,
            board_rpi5.cfg.hole_diameter / 2,
        )
        place_board_holes(
            board_servo_plug.default_hole_positions(board_servo_plug.cfg),
            LOCATION_LEFT_SERVO_PLUG,
            board_servo_plug.cfg.hole_diameter / 2,
        )
        place_board_holes(
            board_servo_plug.default_hole_positions(board_servo_plug.cfg),
            LOCATION_RIGHT_SERVO_PLUG,
            board_servo_plug.cfg.hole_diameter / 2,
        )
        place_board_holes(
            board_relay.default_hole_positions(board_relay.cfg),
            LOCATION_RELAY,
            board_relay.cfg.hole_diameter / 2,
        )
        place_board_holes(
            board_bno055.two_holes_on_one_side(board_bno055.cfg),
            LOCATION_BNO055,
            board_bno055.cfg.hole_diameter / 2,
        )
        place_board_holes(
            board_ina228.two_holes_on_one_side(board_ina228.cfg),
            LOCATION_INA228,
            board_ina228.cfg.hole_diameter / 2,
        )
        place_board_holes(
            board_servo_interface.default_hole_positions(board_servo_interface.cfg),
            LOCATION_SERVO_INTERFACE,
            board_servo_interface.cfg.hole_diameter / 2,
        )
        place_board_holes(
            board_i2c_distributor.default_hole_positions(board_i2c_distributor.cfg),
            LOCATION_I2C_DISTRIBUTOR,
            board_i2c_distributor.cfg.hole_diameter / 2,
        )

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=body_common.THICKNESS)
    return model.part


def main() -> None:
    surface = build_surface()
    model = build_model(surface)

    OUTPUT_DIR.mkdir(exist_ok=True)
    export_step(model, str(OUTPUT_DIR / f"{OUTPUT_NAME}.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(OUTPUT_DIR / f"{OUTPUT_NAME}.dxf"))

    show(model, name=OUTPUT_NAME, clear=True)


if __name__ == "__main__":
    main()