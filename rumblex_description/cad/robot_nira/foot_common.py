#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    Locations,
    Mode,
    Part,
    Polygon,
    Rectangle,
    Sketch,
    add,
    extrude,
)

from common import servo_simplified
from utils.ocp_utils import show

THICKNESS = 1.5
M2_RADIUS = 1.0
M2_5_RADIUS = 1.25

# Shared by the cheek slots, outer armor and assembly; nominal laser-cut fit.
FOOT_SPACER_OVERALL_LENGTH = servo_simplified.BODY_Y + 2 * servo_simplified.CASE_FLANGE_THICKNESS
OUTER_PLATE_HEIGHT = FOOT_SPACER_OVERALL_LENGTH + 2 * THICKNESS
OUTER_PLATE_Y = -24.25
OUTER_TAB_WIDTH = 8.0
OUTER_TAB_X = (28.0, 46.0, 64.0)

# Swept toe and a broad lower rail echo the faceted canopy. Keep the servo
# shoulder and all mechanical interfaces in their original drawing frame.
FOOT_OUTLINE = [
    (-16.25, -27.25),
    (72.0, -27.25),
    (91.75, -16.75),
    (91.75, 0.25),
    (77.0, 2.25),
    (24.0, 9.25),
    (18.75, 9.25),
    (12.75, 15.25),
    (9.75, 15.25),
    (6.75, 12.25),
    (-7.25, -9.75),
    (-12.25, -9.75),
    (-19.25, -16.75),
    (-19.25, -24.25),
]

# Swept gills stay above the spacer bosses and below the upper perimeter.
CHEEK_GILLS = [
    [(x, -13.0), (x + 5, -13.0), (x - 1, -1.0), (x - 6, -1.0)]
    for x in (33.0, 47.0, 61.0)
]

FOOT_MOUNT_HOLES = [
    {"x": 84.75, "y": -8.25, "radius": M2_5_RADIUS},
    {"x": 52.75, "y": -19.75, "radius": M2_5_RADIUS},
    {"x": -14.25, "y": -19.75, "radius": M2_5_RADIUS},
]

TIP_SLOTS = [
    (88.25,  -3.75, 3.0, 4.0),
    (88.25, -12.75, 3.0, 4.0),
]

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Polygon(*FOOT_OUTLINE, align=None)

        for points in CHEEK_GILLS:
            Polygon(*points, align=None, mode=Mode.SUBTRACT)

        for x in OUTER_TAB_X:
            with Locations((x, OUTER_PLATE_Y)):
                Rectangle(OUTER_TAB_WIDTH, THICKNESS, mode=Mode.SUBTRACT)

        for hole in FOOT_MOUNT_HOLES:
            with Locations((hole["x"], hole["y"])):
                Circle(hole["radius"], mode=Mode.SUBTRACT)

        for x, y, width, height in TIP_SLOTS:
            with Locations((x, y)):
                Rectangle(width, height, mode=Mode.SUBTRACT)

    return sketch.sketch

def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)

    return model.part

def main() -> None:
    surface = build_surface()
    result = build_model(surface)
    show(result, name="foot_cutout", clear=True)


if __name__ == "__main__":
    main()
