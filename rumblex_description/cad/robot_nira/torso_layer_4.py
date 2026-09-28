#!/usr/bin/env python3
"""Vented arrowhead canopy ending above the lidar, with rear mounting holes."""

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
    Polygon,
    Sketch,
    add,
    export_step,
    extrude,
)

import robot_nira.torso_common as torso_common
from robot_nira.lidar_layout import canopy_surface
from utils.ocp_utils import show


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(canopy_surface())
        # Paired swept gills leave a solid central spine and a perimeter rim.
        for x in (-78, -62, -46, -30):
            for side in (-1, 1):
                Polygon((x - 5, side * 12), (x + 2, side * 12),
                        (x - 16, side * 39), (x - 23, side * 39),
                        align=None, mode=Mode.SUBTRACT)
        for location in torso_common.hole_locations:
            if location.position.X < 0:
                with Locations(location):
                    Circle(torso_common.hole_radius, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=torso_common.THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "dxf").mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "step/torso_layer_4.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(EXPORT_DIR / "dxf/torso_layer_4.dxf"))

    show(result, name="torso_layer_4", clear=True)


if __name__ == "__main__":
    main()
