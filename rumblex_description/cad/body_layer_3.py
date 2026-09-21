#!/usr/bin/env python3
"""Compact octagonal top plate with the shared opening, slots, and spacer holes."""

from pathlib import Path

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Sketch,
    add,
    export_step,
    extrude,
)

import body_common
from utils.ocp_utils import show


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        # Shared 220 x 130 mm rectangle with 20 mm corner chamfers.
        # This leaves 3.25 mm beyond the shared spacer footprints.
        add(body_common.base_plate.sketch)
        add(body_common.hantel.sketch, mode=Mode.SUBTRACT)

        for location in body_common.rectangle_slots_locations:
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

    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/body_layer_3.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/body_layer_3.dxf")

    show(result, name="body_layer_3", clear=True)


if __name__ == "__main__":
    main()
