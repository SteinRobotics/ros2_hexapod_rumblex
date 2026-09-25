#!/usr/bin/env python3
"""Vented outer foot armor, with three through-tabs per cheek plate.

Flat sketch X follows the foot; Y becomes assembly Z. Tabs are nominal
8 x 1.5 mm, without laser kerf compensation, like the chassis joints.
"""

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    BuildPart, BuildSketch, ExportDXF, Locations, Mode, Part, Polygon,
    Pos, Rectangle, Rot, Sketch, add, export_step, extrude,
)

from robot_nira import EXPORT_DIR
from robot_nira.foot_common import (
    THICKNESS, OUTER_PLATE_HEIGHT, OUTER_PLATE_Y, OUTER_TAB_WIDTH, OUTER_TAB_X,
)
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show


def build_surface() -> Sketch:
    h = OUTER_PLATE_HEIGHT
    with BuildSketch() as sketch:
        # Clipped heel and pointed nose frame a continuous longitudinal spine.
        Polygon((20, 8), (24, THICKNESS), (70, THICKNESS),
                (80, h / 2), (70, h - THICKNESS),
                (24, h - THICKNESS), (20, h - 8), align=None)
        for x in OUTER_TAB_X:
            with Locations((x, THICKNESS / 2), (x, h - THICKNESS / 2)):
                Rectangle(OUTER_TAB_WIDTH, THICKNESS)
        for x in (35, 49, 63):
            for side in (-1, 1):
                Polygon((x, h / 2 + side * 4),
                        (x + 5, h / 2 + side * 4),
                        (x - 1, h / 2 + side * 11),
                        (x - 6, h / 2 + side * 11),
                        align=None, mode=Mode.SUBTRACT)
    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)
    return model.part


def build_placed_model() -> Part:
    # Rotation sends extrusion toward -Y; center it on the cheek slot row.
    plate = (Pos(0, OUTER_PLATE_Y + THICKNESS / 2, 0)
             * Rot(X=90) * build_model(build_surface()))
    plate.label = "foot_outer"
    plate.color = COLOR_CREAMY_WHITE
    return plate


def main() -> None:
    surface = build_surface()
    model = build_model(surface)
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "dxf").mkdir(parents=True, exist_ok=True)
    export_step(model, str(EXPORT_DIR / "step/foot_outer.step"))
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.write(str(EXPORT_DIR / "dxf/foot_outer.dxf"))
    show(model, name="foot_outer", clear=True)


if __name__ == "__main__":
    main()
