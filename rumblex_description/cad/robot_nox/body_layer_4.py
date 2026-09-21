#!/usr/bin/env python3
"""Compact octagonal top plate with the shared opening, slots, and spacer holes."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nox import EXPORT_DIR

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

import robot_nox.body_common as body_common
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

    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "body_layer_4.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(EXPORT_DIR / "body_layer_4.dxf"))

    show(result, name="body_layer_4", clear=True)


if __name__ == "__main__":
    main()
