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
from build123d.exporters import ColorIndex
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

OUTPUT_DIR = EXPORT_DIR
OUTPUT_NAME = "torso_layer_0"
BOARD_LABEL_FONT_SIZE = 2.5  # mm


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
        insert(torso_common.base_plate)

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
        insert(surface)
        extrude(amount=torso_common.THICKNESS)
    return model.part


def build_board_labels() -> list[Sketch]:
    """Place engraving outlines at each board center, following its rotation."""
    return [
        Pos(mount.placement.x, mount.placement.y)
        * Rot(Z=mount.placement.rotation)
        * Text(mount.name, font_size=BOARD_LABEL_FONT_SIZE,
               font="Arial", align=(Align.CENTER, Align.CENTER))
        for mount in BOARD_MOUNTS
    ]


def build_dxf(surface: Sketch) -> ExportDXF:
    """Keep board names on a blue engraving layer separate from the cuts."""
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.add_layer("ENGRAVE", color=ColorIndex.BLUE)
    for label in build_board_labels():
        # Include inner glyph contours as well (e.g. the counters in B and O).
        drawing.add_shape(label, layer="ENGRAVE")
    return drawing


def main() -> None:
    surface = build_surface()
    model = build_model(surface)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / "step").mkdir(parents=True, exist_ok=True)
    dxf_dir = dxf_directory(torso_common.THICKNESS, export_dir=OUTPUT_DIR)
    export_step(model, str(OUTPUT_DIR / "step" / f"{OUTPUT_NAME}.step"))

    dxf_export = build_dxf(surface)
    dxf_export.write(str(dxf_dir / f"{OUTPUT_NAME}.dxf"))

    show(model, name=OUTPUT_NAME, clear=True)


if __name__ == "__main__":
    main()
