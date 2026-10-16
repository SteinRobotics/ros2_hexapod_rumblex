#!/usr/bin/env python3
"""Vented arrowhead canopy with a locating frame on its underside."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR, dxf_directory

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    Compound,
    ExportDXF,
    Face,
    Kind,
    Locations,
    Mode,
    Part,
    Polygon,
    Sketch,
    insert,
    export_step,
    extrude,
    offset,
)

import robot_nira.torso_common as torso_common
import robot_nira.torso_layer_3 as torso_layer_3
from robot_nira.lidar_layout import canopy_surface
from utils.ocp_utils import show


FRAME_THICKNESS = 3.0
FRAME_WIDTH = 5.0


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        insert(canopy_surface())
        # Paired swept gills leave a solid central spine and a perimeter rim.
        for x in (-60, -44, -28, -12):
            for side in (-1, 1):
                Polygon((x - 5, side * 12), (x + 2, side * 12),
                        (x - 16, side * 39), (x - 23, side * 39),
                        align=None, mode=Mode.SUBTRACT)
        for location in torso_common.hole_locations:
            if location.position.X < 0:
                with Locations(location):
                    Circle(torso_common.hole_radius, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_roof_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        insert(surface)
        extrude(amount=torso_common.THICKNESS)

    return model.part


def build_frame_surface() -> Sketch:
    """Match layer 3's central opening with a constant-width inner rim."""
    support = torso_layer_3.build_surface().faces()[0]
    opening = max((Face(wire) for wire in support.inner_wires()), key=lambda face: face.area)
    outline = Sketch([opening])
    inner = offset(outline, amount=-FRAME_WIDTH, kind=Kind.INTERSECTION)
    with BuildSketch() as sketch:
        insert(outline)
        insert(inner, mode=Mode.SUBTRACT)
    return sketch.sketch


def build_frame_model(surface: Sketch) -> Part:
    """Seat the frame's top against the roof underside at local Z=0."""
    with BuildPart() as model:
        insert(surface)
        extrude(amount=-FRAME_THICKNESS)
    return model.part


def build_model(surface: Sketch) -> Compound:
    roof = build_roof_model(surface)
    roof.label = "torso_layer_4_roof"
    frame = build_frame_model(build_frame_surface())
    frame.label = "torso_layer_4_frame"
    return Compound(label="torso_layer_4", children=[roof, frame])


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    dxf_dir = dxf_directory(torso_common.THICKNESS)
    export_step(result, str(EXPORT_DIR / "step/torso_layer_4.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(dxf_dir / "torso_layer_4.dxf"))

    frame_surface = build_frame_surface()
    frame_dxf = ExportDXF()
    frame_dxf.add_shape(frame_surface)
    frame_dxf.write(str(dxf_directory(FRAME_THICKNESS) / "torso_layer_4_frame.dxf"))

    show(result, name="torso_layer_4", clear=True)


if __name__ == "__main__":
    main()
