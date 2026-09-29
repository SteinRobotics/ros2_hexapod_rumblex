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
    Pos,
    Rectangle,
    Sketch,
    Sphere,
    add,
    extrude,
)

from common import servo_simplified
from robot_nira.armor_style import gill_points
from utils.colors import COLOR_RED
from utils.ocp_utils import show

THICKNESS = 1.5
M2_RADIUS = 1.0
M2_5_RADIUS = 1.25

# Shared by the cheek slots, outer armor and assembly; nominal laser-cut fit.
FOOT_SPACER_OVERALL_LENGTH = servo_simplified.BODY_Y + 2 * servo_simplified.CASE_FLANGE_THICKNESS
CHEEK_OUTER_SPAN = FOOT_SPACER_OVERALL_LENGTH + 2 * THICKNESS
OUTER_PLATE_HEIGHT = CHEEK_OUTER_SPAN
OUTER_PLATE_Y = -24.25
OUTER_TAB_WIDTH = 8.0
OUTER_TAB_X = (28.0, 46.0, 64.0)

# Swept toe and a broad lower rail echo the faceted canopy. Keep the servo
# shoulder and all mechanical interfaces in their original drawing frame.
FOOT_OUTLINE = [
    (-16.25, -27.25),    #0 
    (72.0, -27.25),      #1
    (91.75, -16.75),     #2
    (91.75, 0.25),       #3
    (77.0, 2.25),        #4
    (18.75, 7.0),       #5
    (12.75, 15.25),      #6
    (9.75, 15.25),       #7
    (6.75, 12.25),       #8
    (-7.25, -9.75),      #9
    (-12.25, -9.75),     #10
    (-19.25, -16.75),    #11
    (-19.25, -24.25),    #12
]

# Preview only: zero-based FOOT_OUTLINE indices, e.g. (0, 4, 13).
# A single point needs a trailing comma: (0,). Use () to disable markers.
# Alternatively run: python robot_nira/foot_common.py --marker 0 --marker 4
DEBUG_OUTLINE_POINTS = (5,)

FOOT_OUTLINE_OLD = [
    (-16.25, -25.75),  # 0
    ( 53.75, -25.75),  # 1
    ( 91.75, -16.75),  # 2
    ( 91.75,   0.25),  # 3
    ( 18.75,   9.25),  # 4
    ( 12.75,  15.25),  # 5
    (  9.75,  15.25),  # 6
    (  6.75,  12.25),  # 7
    ( -7.25,  -9.75),  # 8
    (-12.25,  -9.75),  # 9
    (-19.25, -16.75),  # 10
    (-19.25, -22.75),  # 11
]

# Swept gills stay above the spacer bosses and below the upper perimeter.
CHEEK_GILLS = [
    gill_points(x, -13.0, 7.0)
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
    import argparse

    parser = argparse.ArgumentParser(description="Preview the Nira foot outline.")
    parser.add_argument(
        "--marker", type=int, action="append", choices=range(len(FOOT_OUTLINE)),
        help="Mark a zero-based FOOT_OUTLINE point (repeat for multiple points).",
    )
    args = parser.parse_args()
    indices = args.marker if args.marker is not None else DEBUG_OUTLINE_POINTS
    for index in indices:
        if not 0 <= index < len(FOOT_OUTLINE):
            parser.error(f"outline marker index {index} is out of range")

    surface = build_surface()
    result = build_model(surface)
    show(result, name="foot_cutout", clear=True)
    for index in indices:
        x, y = FOOT_OUTLINE[index]
        marker = Pos(x, y, THICKNESS) * Sphere(1.0)
        marker.color = COLOR_RED
        show(marker, name=f"outline_{index} ({x}, {y})", clear=False)


if __name__ == "__main__":
    main()
