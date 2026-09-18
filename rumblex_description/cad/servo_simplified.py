#!/usr/bin/env python3
"""Simplified HX-35H servo body — outer shell only, no internal parts.

"""

from pathlib import Path

from build123d import (
    Box,
    BuildPart,
    Cylinder,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    export_step,
    RevoluteJoint,
    Axis,
)

from utils.ocp_utils import show

from utils.colors import COLOR_DARK_GRAY

from servo_cutouts import *

# ── Outer-body dimensions (mm) ───────────────────────────────────────────────
# Main rectangular case
BODY_W = 25.00  # X width
BODY_Y = 29.00   # Y depth
BODY_H  = 45.00   # total height (Z); base rests on Z = 0


# Servo horn Z centre
HORN_Z_CTR = 35.00

# Servo horn dimensions (cylindrical protrusions at the front and back)
HORN_R       = 10.00
HORN_FRONT_Y = 3.50
HORN_BACK_Y  = 3.00
HORN_DISTANCE_TO_BODY = 0.75

# M2 mounting holes (through the servo body in the Y direction)
M2_R        = 1.00
HOLE_X      = 10.25
HOLE_Z_LOW  = 2.60
HOLE_Z_HIGH = 27.05


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
        axis=Axis((0, 0, HORN_Z_CTR), (0, 1, 0))
        )

        # 4 - add the front horn cutout with a 1.5 mm depth
        plane = Plane(origin=(0.0, -0.75, 14.00), x_dir=(-1, 0, 0), z_dir=(0, 1, 0))
        with Locations(Location(plane)):
            Box(BODY_W, 28.00, 1.5)
            
        # 5 - add the back horn cutout with a 1.5 mm depth
        plane = Plane(origin=(0.0, BODY_Y + 0.75, 14.00), x_dir=(-1, 0, 0), z_dir=(0, 1, 0))
        with Locations(Location(plane)):
            Box(BODY_W, 28.00, 1.5)

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
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/servo_simplified_HX35H.step")
    show(result, name="servo_simplified_HX35H", clear=True)


if __name__ == "__main__":
    main()
