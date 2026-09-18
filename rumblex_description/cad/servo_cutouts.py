#!/usr/bin/env python3

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    Locations,
    Mode,
    Part,
    Polygon,
    Sketch,
    add,
    extrude,
)

from utils.geometry_utils import mirror_points_vertical_axis as mirror_y
from utils.ocp_utils import show

from servo_with_connections import SERVO_CONNECTIONS_FLAT

THICKNESS = 1.5
M2_RADIUS = 1.0
SERVO_CUTOUT_HIGHT = 15.0


SERVO_BACK_CUTOUT_RIGHT = [
    (  9.75,  -9.75),
    (  9.75,   2.25),
    ( 10.75,   2.25),
    ( 10.75,   9.25),
    (  6.75,   9.25),
    (  6.75,  11.5),
    (  10.0,  SERVO_CUTOUT_HIGHT),
    (  15.0,  SERVO_CUTOUT_HIGHT),
    (  15.0,  18.0),
]

SERVO_BACK_CUTOUT = (
    SERVO_BACK_CUTOUT_RIGHT
    + mirror_y(list(reversed(SERVO_BACK_CUTOUT_RIGHT)))
)

SERVO_FRONT_CUTOUT_RIGHT = [
    ( 7.25, -13.75),
    (  7.25,  11.5),
    (  10.0,  SERVO_CUTOUT_HIGHT),
    (  15.0,  SERVO_CUTOUT_HIGHT),
    (  15.0,  18.0),
]

SERVO_FRONT_CUTOUT = (
    SERVO_FRONT_CUTOUT_RIGHT
    + mirror_y(list(reversed(SERVO_FRONT_CUTOUT_RIGHT)))
)

OUTLINE_POINTS_FOR_TESTING = [
    (-15,  -15), 
    (  15,  -15), 
    (  15,   15),  
    ( -15,   15), 
]

# Rectangle corners for the servo bracket holes; center is the drawing origin.
# Holes are placed at the first three corners (top-right, bottom-right, bottom-left).
# SERVO_BRACKET_RECT = [
#     ( 10.25,  12.25),  # top-right
#     ( 10.25, -12.25),  # bottom-right
#     (-10.25, -12.25),  # bottom-left
#     (-10.25,  12.25),  # top-left (no hole)
# ]

SERVO_BRACKET_HOLES = [
    (x, y, M2_RADIUS) for x, y in SERVO_CONNECTIONS_FLAT
]

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Polygon(*OUTLINE_POINTS_FOR_TESTING)
        Polygon(*SERVO_FRONT_CUTOUT, mode=Mode.SUBTRACT)

        for x, y, radius in SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

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
