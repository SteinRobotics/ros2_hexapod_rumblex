#!/usr/bin/env python3
"""Simplified HX-35H servo body and shared mounting cutouts.

"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Cylinder,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    Polygon,
    Sketch,
    add,
    extrude,
    RevoluteJoint,
    Axis,
)

from common.export_utils import export_common_part

from utils.geometry_utils import mirror_points_vertical_axis as mirror_y
from utils.ocp_utils import show

from utils.colors import COLOR_DARK_GRAY

# ── Outer-body dimensions (mm) ───────────────────────────────────────────────
# Main rectangular case
BODY_W = 25.00  # X width
BODY_Y = 29.00   # Y depth
BODY_H  = 45.00   # total height (Z); base rests on Z = 0
CASE_FLANGE_THICKNESS = 1.5
CASE_FLANGE_HEIGHT = 28.0


# Servo horn Z centre
HORN_Z_CTR = 35.00

# Servo horn dimensions (cylindrical protrusions at the front and back)
HORN_R       = 10.00
HORN_FRONT_Y = 3.50
HORN_BACK_Y  = 3.00
HORN_DISTANCE_TO_BODY = 0.75
HORN_FRONT_FACE_Y = -HORN_DISTANCE_TO_BODY - HORN_FRONT_Y

# Shared by the servo and assemblies containing it. The joint origin is on
# the horn's outer mounting face, where the bracket's inner face touches it.
HORN_AXIS = Axis((0, HORN_FRONT_FACE_Y, HORN_Z_CTR), (0, 1, 0))

# M2 mounting holes (through the servo body in the Y direction)
M2_R        = 1.00
HOLE_X      = 10.25
HOLE_Z_LOW  = 2.60
HOLE_Z_HIGH = 27.05


# Shared foot/body cutout profiles in drawing coordinates (mm).
CUTOUT_PREVIEW_THICKNESS = 1.5
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

SERVO_BRACKET_HOLES = [
    (x, y, M2_R)
    for x in (-HOLE_X, HOLE_X)
    for y in (-(HOLE_Z_HIGH - HOLE_Z_LOW) / 2, (HOLE_Z_HIGH - HOLE_Z_LOW) / 2)
]


def build_cutout_surface() -> Sketch:
    """Build a sample plate sketch with the front cutout and mounting holes."""
    with BuildSketch() as sketch:
        Polygon(*OUTLINE_POINTS_FOR_TESTING)
        Polygon(*SERVO_FRONT_CUTOUT, align=None, mode=Mode.SUBTRACT)

        for x, y, radius in SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

    return sketch.sketch

def build_cutout_model(surface: Sketch) -> Part:
    """Extrude a cutout preview sketch into a sample plate."""
    with BuildPart() as model:
        add(surface)
        extrude(amount=CUTOUT_PREVIEW_THICKNESS)

    return model.part


# ── Model ────────────────────────────────────────────────────────────────────

def build_model() -> Part:
    with BuildPart() as p:
        # 1 · main rectangular case
        with Locations([(0, BODY_Y / 2, BODY_H / 2)]):
            Box(BODY_W, BODY_Y, BODY_H)

        # 2 - servo horn front
        plane = Plane(origin=(0.0, -(HORN_FRONT_Y / 2) - HORN_DISTANCE_TO_BODY , HORN_Z_CTR), z_dir=(0, 1, 0))
        with Locations(Location(plane)):
            Cylinder(HORN_R, HORN_FRONT_Y, mode=Mode.ADD)
            Cylinder(1.0, HORN_FRONT_Y, mode=Mode.SUBTRACT)

        # 3 - servo horn back
        plane = Plane(origin=(0.0, BODY_Y + HORN_BACK_Y / 2 + HORN_DISTANCE_TO_BODY, HORN_Z_CTR), z_dir=(0, 1, 0))
        with Locations(Location(plane)):
            Cylinder(HORN_R, HORN_BACK_Y, mode=Mode.ADD)
            Cylinder(1.0, HORN_BACK_Y, mode=Mode.SUBTRACT)
            
        # Servo horn rotation axis
        RevoluteJoint(
            "rotation",
            axis=HORN_AXIS,
        )

        # 4 - add the front horn cutout with a 1.5 mm depth
        plane = Plane(origin=(0, -CASE_FLANGE_THICKNESS / 2, CASE_FLANGE_HEIGHT / 2), x_dir=(-1, 0, 0), z_dir=(0, 1, 0))
        with Locations(Location(plane)):
            Box(BODY_W, CASE_FLANGE_HEIGHT, CASE_FLANGE_THICKNESS)
            
        # 5 - add the back horn cutout with a 1.5 mm depth
        plane = Plane(origin=(0, BODY_Y + CASE_FLANGE_THICKNESS / 2, CASE_FLANGE_HEIGHT / 2), x_dir=(-1, 0, 0), z_dir=(0, 1, 0))
        with Locations(Location(plane)):
            Box(BODY_W, CASE_FLANGE_HEIGHT, CASE_FLANGE_THICKNESS)

        # 8 - M2 mounting holes (through in Y, spanning back-plate to front-case)
        hole_depth = BODY_Y + 4.0   # +2 ensures clean Boolean cut
        hole_y_ctr = BODY_Y / 2
        for z in (HOLE_Z_LOW, HOLE_Z_HIGH):
            for x in (HOLE_X, -HOLE_X):
                plane = Plane(origin=(x, hole_y_ctr, z), z_dir=(0, 1, 0))
                with Locations(Location(plane)):
                    Cylinder(M2_R, hole_depth, mode=Mode.SUBTRACT)


    return p.part


def main() -> None:
    result = build_model()
    result.color = COLOR_DARK_GRAY
    export_common_part(result, "servo_simplified_HX35H")
    show(result, name="servo_simplified_HX35H", clear=True)


if __name__ == "__main__":
    main()
