#!/usr/bin/env python3
"""Swept canopy frame with a solid nose above the lidar."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Pos,
    Rectangle,
    Sketch,
    add,
    export_step,
    extrude,
)

import robot_nira.body_common as body_common
from robot_nira.lidar_layout import canopy_surface
from utils.ocp_utils import show


def build_surface() -> Sketch:
    opening = body_common.hantel.sketch & (Pos(-100, 0) * Rectangle(240, 200))
    with BuildSketch() as sketch:
        # Keep the rear service opening, but bridge its front so the lower
        # frame has the same pointed nose as the roof above it.
        add(canopy_surface())
        add(opening, mode=Mode.SUBTRACT)

        for location in [*body_common.rectangle_slots_locations, *body_common.diagonal_slots_locations]:
            with Locations(location):
                add(body_common.rectangle_slots.sketch, mode=Mode.SUBTRACT)

        for location in body_common.hole_locations:
            with Locations(location):
                Circle(body_common.hole_radius, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=body_common.THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "body_layer_3.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(EXPORT_DIR / "body_layer_3.dxf"))

    show(result, name="body_layer_3", clear=True)


if __name__ == "__main__":
    main()
