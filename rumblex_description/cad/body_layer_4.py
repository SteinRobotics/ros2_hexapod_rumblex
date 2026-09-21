#!/usr/bin/env python3
"""Compact octagonal top plate with the shared opening, slots, and spacer holes."""

from pathlib import Path

from build123d import (
    BuildPart,
    BuildSketch,
    ExportDXF,
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
        add(body_common.base_plate.sketch)

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
    export_step(result, "generated/body_layer_4.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/body_layer_4.dxf")

    show(result, name="body_layer_4", clear=True)


if __name__ == "__main__":
    main()
