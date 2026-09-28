#!/usr/bin/env python3
"""Body assembly with the 7 coxa servos placed at their mounting positions."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import math
from robot_nira import EXPORT_DIR

from build123d import Compound, Pos, Rot, export_step, export_stl

from robot_nira.board_layout import Placement, BOARD_MOUNTS
from robot_nira import body_layout
from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show

import robot_nira.assembly_body as assembly_body
import robot_nira.body_common as body_common
import common.servo_simplified as servo_simplified


servo_by_name = {}

# Z height of the servo local origin so that:
#   front horn (local Y < 0) sits inside body layer 2
#   back  horn (local Y > BODY_Y) sits inside body layer 1
SERVO_Z = body_layout.LAYER_2_CENTER - (
    servo_simplified.HORN_FRONT_Y / 2 + servo_simplified.HORN_DISTANCE_TO_BODY
)
# Midpoint of the two M2 hole rows along the servo's local Z axis.
# Used to centre the servo over the body-layer cutout.
SERVO_Z_MID = (servo_simplified.HOLE_Z_LOW + servo_simplified.HOLE_Z_HIGH) / 2


def servo_location(config: body_common.ServoCutoutConfig, z: float = SERVO_Z):
    """Return the world transform for a servo at a body cutout.

    ``config.offset_*`` locates the centre of the cutout, whereas a servo's
    local origin is at the bottom of the case. Keeping that correction
    here makes every consumer use the same convention.
    """
    angle = config.rotation_deg_clockwise
    angle_rad = math.radians(angle)
    x = config.offset_x - SERVO_Z_MID * math.sin(angle_rad)
    y = config.offset_y - SERVO_Z_MID * math.cos(angle_rad)
    return Pos(x, y, z) * Rot(0, 0, -angle) * Rot(-90, 0, 0)


def place_board(board_module, placement: Placement):
    """Build a board-with-spacers assembly and place it (component-side down) on the body.

    The board is flipped and rotated around its own center first, then
    moved to the placement's XY offset - matching the hole pattern cut in
    body_layer_0.build_surface().
    """
    board = board_module.build_board_with_spacers()
    z_pos = board_module.cfg.thickness + board_module.cfg.spacer_height + body_common.THICKNESS
    return (
        Pos(placement.x, placement.y, z_pos)
        * Rot(0, 0, placement.rotation)
        * Rot(0, 180, 0)
        * board
    )


def build_assembly() -> Compound:
    # This module-level mapping is a convenience for parent assemblies.  Do
    # not retain instances from an earlier build.
    servo_by_name.clear()
    body = assembly_body.build_assembly()

    # add servos
    servo_part = servo_simplified.build_model()
    servo_part.color = COLOR_DARK_GRAY

    servo_instances = []
    for name, config in body_common.SERVO_CUTOUT_CONFIGS.items():
        instance = servo_location(config) * servo_part
        instance.label = f"servo_{name}"
        servo_instances.append(instance)
        servo_by_name[f"servo_{name}"] = instance

    # add pcbs
    boards = [place_board(mount.module, mount.placement) for mount in BOARD_MOUNTS]

    return Compound(
        label="assembly_body_servo",
        children=[
            body,
            *servo_instances,
            *boards,
        ],
    )


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "stl").mkdir(parents=True, exist_ok=True)
    # export_step(assembly, str(EXPORT_DIR / "step/assembly_body_servo.step"))
    export_stl(assembly, str(EXPORT_DIR / "stl/assembly_body_servo.stl"))
    show(assembly, name="assembly_body_servo", clear=True)


if __name__ == "__main__":
    main()
