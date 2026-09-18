#!/usr/bin/env python3

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

from utils.ocp_utils import show

THICKNESS = 1.5
M2_RADIUS = 1.0
M2_5_RADIUS = 1.25

FOOT_OUTLINE = [
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
        Polygon(*FOOT_OUTLINE)

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
