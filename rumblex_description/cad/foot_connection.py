#!/usr/bin/env python3

from pathlib import Path

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Rectangle,
    Sketch,
    add,
    export_step,
    extrude,
)

from utils.ocp_utils import show

THICKNESS = 3.0

# The updated imported/leg_connection.svg now contains a compact connection
# plate profile that is best expressed with rectangle booleans.
OUTER_WIDTH = 35.0
OUTER_HEIGHT = 13.0
SIDE_NOTCH_WIDTH = 1.5
SIDE_NOTCH_HEIGHT = 5.0
CENTER_HOLE_RADIUS = 0.5
CENTER_HOLE_X = 0.0
CENTER_HOLE_Y = 0.0


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Rectangle(OUTER_WIDTH, OUTER_HEIGHT)

        # Side notches match the SVG path's stepped profile.
        notch_center_x = OUTER_WIDTH / 2 - SIDE_NOTCH_WIDTH / 2
        with Locations((-notch_center_x, 0), (notch_center_x, 0)):
            Rectangle(SIDE_NOTCH_WIDTH, SIDE_NOTCH_HEIGHT, mode=Mode.SUBTRACT)

        with Locations((CENTER_HOLE_X, CENTER_HOLE_Y)):
            Circle(CENTER_HOLE_RADIUS, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/foot_connection.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/foot_connection.dxf")

    show(result, name="foot_connection", clear=True)


if __name__ == "__main__":
    main()
