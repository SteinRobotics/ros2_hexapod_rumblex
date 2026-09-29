#!/usr/bin/env python3
"""Body layer 0: base plate with chamfered corner cutouts and PCB mounting holes."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from collections.abc import Iterable
from robot_nira import EXPORT_DIR, dxf_directory

from build123d import *
from utils.ocp_utils import show

import robot_nira.torso_common as torso_common
from robot_nira.board_layout import Placement, BOARD_MOUNTS

# orientation
#         ^ x
#         |
#         |
#    <----x
#    y
#


TOE_MOUNTING_POSITIONS = [(sx * 75.0, sy * 37.0) for sx, sy in
                          ((1, 1), (1, -1), (-1, -1), (-1, 1))]
TOE_HOLE_RADIUS = 1.5
TOE_MOUNTING_HOLES = [(x, y, TOE_HOLE_RADIUS) for x, y in TOE_MOUNTING_POSITIONS]


OCTAGON_POSITIONS = [
    torso_common.octagon_position_left_top,
    torso_common.octagon_position_right_top,
    torso_common.octagon_position_right_bottom,
    torso_common.octagon_position_left_bottom,
]

OUTPUT_DIR = EXPORT_DIR
OUTPUT_NAME = "torso_layer_0"


# TODO: move to generic helper file or to torso_common.py
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
        add(torso_common.base_plate)

        for pos in OCTAGON_POSITIONS:
            with Locations(pos):
                add(chamfered_octagon(55, 55, 4))

        for loc in torso_common.hole_locations:
            with Locations(loc):
                Circle(torso_common.hole_radius, mode=Mode.SUBTRACT)

        for x, y, radius in TOE_MOUNTING_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for mount in BOARD_MOUNTS:
            place_board_holes(mount.holes(), mount.placement,
                              mount.module.cfg.hole_diameter / 2)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=torso_common.THICKNESS)
    return model.part


def main() -> None:
    surface = build_surface()
    model = build_model(surface)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / "step").mkdir(parents=True, exist_ok=True)
    dxf_dir = dxf_directory(torso_common.THICKNESS, export_dir=OUTPUT_DIR)
    export_step(model, str(OUTPUT_DIR / "step" / f"{OUTPUT_NAME}.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(dxf_dir / f"{OUTPUT_NAME}.dxf"))

    show(model, name=OUTPUT_NAME, clear=True)


if __name__ == "__main__":
    main()
