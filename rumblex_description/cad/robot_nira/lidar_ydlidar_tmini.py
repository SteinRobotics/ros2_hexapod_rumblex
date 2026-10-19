#!/usr/bin/env python3
"""Simplified YDLIDAR T-mini envelope for robot assemblies (millimetres).

Reference: YDLIDAR T-mini Plus Data Sheet V1.1, Fig. 2:
https://www.ydlidar.com/Public/upload/files/2024-05-24/YDLIDAR%20T-mini%20Plus%20Data%20Sheet_V1.1%20(240131).pdf
Manufacturer drawing mirrored at:
https://akizukidenshi.com/goodsaffix/YDLIDAR%20T-mini%20Plus%20Data%20Sheet_V1.1%20(240131).pdf

The housing is centred on XY with its mounting face at Z=0. The scanner
axis is +Z. Overall size, scanner diameter, housing height and scan height
follow the drawing; the housing corner radius is an approximation. Mounting
bores follow the drawing (threads are not modelled). The front GH1.25-4P
socket follows Fig. 2's opening width and bottom elevation and section 2.3's
contact pitch/count; undimensioned socket details are approximate. Case taper
and rotor details are omitted. This is an assembly envelope, not a manufacturing model.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Cylinder,
    Location,
    Locations,
    Mode,
    Part,
    RectangleRounded,
    RigidJoint,
    export_step,
    extrude,
)

from utils.colors import COLOR_BRASS, COLOR_CREAMY_WHITE, COLOR_DARK_GRAY
from utils.ocp_utils import show


BODY_WIDTH = 38.6
BODY_DEPTH = 38.6
BODY_HEIGHT = 21.15
BODY_CORNER_RADIUS = 2.0  # Simplified, not dimensioned in the drawing.
TOTAL_HEIGHT = 33.9
SCANNER_DIAMETER = 36.2
SCANNER_HEIGHT = TOTAL_HEIGHT - BODY_HEIGHT
SCAN_HEIGHT = 26.3

# Fig. 2, View A is viewed from below: its horizontal direction is reversed
# here when looking down +Z from above. The connector side is local -Y.
MOUNT_HOLE_SPACING = 28.2
MOUNT_HOLE_POSITIONS = tuple(
    (side * MOUNT_HOLE_SPACING / 2, -side * MOUNT_HOLE_SPACING / 2)
    for side in (-1, 1)
)
MOUNT_HOLE_DIAMETER = 2.5
MOUNT_BOTTOM_COUNTERBORE_DIAMETER = 4.8
MOUNT_BOTTOM_COUNTERBORE_DEPTH = 12.3
MOUNT_TOP_COUNTERBORE_DIAMETER = 3.58
MOUNT_TOP_COUNTERBORE_DEPTH = 5.2
# Separate blind hole, not a third screw mounting point.
LOCATING_HOLE_POSITION = (15.7, 15.7)
LOCATING_HOLE_DIAMETER = 2.5
LOCATING_HOLE_DEPTH = 7.2

# Fig. 2: connector opening width and its lower edge above the mounting face.
CONNECTOR_WIDTH = 8.7
CONNECTOR_BOTTOM = 2.4
CONNECTOR_CONTACT_PITCH = 1.25  # Section 2.3: GH1.25-4P.
CONNECTOR_CONTACT_COUNT = 4
# Undimensioned details estimated from the front view, not a mating-part model.
CONNECTOR_HEIGHT = 4.5
CONNECTOR_DEPTH = 2.5
CONNECTOR_RECESS = 0.3
CONNECTOR_CLEARANCE = 0.3
CONNECTOR_WALL = 0.45
CONNECTOR_CONTACT_SIZE = 0.3
CONNECTOR_Z = CONNECTOR_BOTTOM + CONNECTOR_HEIGHT / 2
CONNECTOR_FACE_Y = -BODY_DEPTH / 2 + CONNECTOR_RECESS


def build_model() -> Part:
    """Return a single solid with mounting, scan and cable-connection joints."""
    with BuildPart() as model:
        with BuildSketch():
            RectangleRounded(BODY_WIDTH, BODY_DEPTH, BODY_CORNER_RADIUS)
        extrude(amount=BODY_HEIGHT)

        with Locations((0, 0, BODY_HEIGHT)):
            Cylinder(
                SCANNER_DIAMETER / 2,
                SCANNER_HEIGHT,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )

        with Locations(*MOUNT_HOLE_POSITIONS):
            Cylinder(
                MOUNT_HOLE_DIAMETER / 2, BODY_HEIGHT,
                align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT,
            )
            Cylinder(
                MOUNT_BOTTOM_COUNTERBORE_DIAMETER / 2, MOUNT_BOTTOM_COUNTERBORE_DEPTH,
                align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT,
            )
        with Locations(*[(x, y, BODY_HEIGHT - MOUNT_TOP_COUNTERBORE_DEPTH)
                         for x, y in MOUNT_HOLE_POSITIONS]):
            Cylinder(
                MOUNT_TOP_COUNTERBORE_DIAMETER / 2, MOUNT_TOP_COUNTERBORE_DEPTH,
                align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT,
            )
        with Locations(LOCATING_HOLE_POSITION):
            Cylinder(
                LOCATING_HOLE_DIAMETER / 2, LOCATING_HOLE_DEPTH,
                align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT,
            )

        # Pocket in the front housing, with a recessed socket joined at its back.
        with Locations((0, -BODY_DEPTH / 2 + CONNECTOR_DEPTH / 2, CONNECTOR_Z)):
            Box(CONNECTOR_WIDTH, CONNECTOR_DEPTH, CONNECTOR_HEIGHT, mode=Mode.SUBTRACT)
        socket_depth = CONNECTOR_DEPTH - CONNECTOR_RECESS + CONNECTOR_WALL
        socket_width = CONNECTOR_WIDTH - 2 * CONNECTOR_CLEARANCE
        socket_height = CONNECTOR_HEIGHT - 2 * CONNECTOR_CLEARANCE
        with Locations((0, CONNECTOR_FACE_Y + socket_depth / 2, CONNECTOR_Z)):
            Box(socket_width, socket_depth, socket_height)
        cavity_depth = CONNECTOR_DEPTH - CONNECTOR_RECESS - CONNECTOR_WALL
        with Locations((0, CONNECTOR_FACE_Y + cavity_depth / 2, CONNECTOR_Z)):
            Box(socket_width - 2 * CONNECTOR_WALL, cavity_depth,
                socket_height - 2 * CONNECTOR_WALL, mode=Mode.SUBTRACT)
        for index in range(CONNECTOR_CONTACT_COUNT):
            x = (index - (CONNECTOR_CONTACT_COUNT - 1) / 2) * CONNECTOR_CONTACT_PITCH
            with Locations((x, CONNECTOR_FACE_Y + cavity_depth / 2 + 0.2, CONNECTOR_Z)):
                Box(CONNECTOR_CONTACT_SIZE, cavity_depth, CONNECTOR_CONTACT_SIZE)

        RigidJoint("connector", joint_location=Location(
            (0, CONNECTOR_FACE_Y, CONNECTOR_Z), (90, 0, 0)))
        RigidJoint("mount", joint_location=Location((0, 0, 0)))
        for index, (x, y) in enumerate(MOUNT_HOLE_POSITIONS, start=1):
            RigidJoint(f"mount_{index}", joint_location=Location((x, y, 0)))
        RigidJoint("locating", joint_location=Location((*LOCATING_HOLE_POSITION, 0)))
        RigidJoint("scan", joint_location=Location((0, 0, SCAN_HEIGHT)))

    result = model.part
    result.label = "lidar_ydlidar_tmini"
    result.color = COLOR_DARK_GRAY
    for face in result.faces():
        center = face.center()
        if (abs(center.X) < socket_width / 2 + 1e-6
                and CONNECTOR_FACE_Y - 1e-6 <= center.Y < CONNECTOR_FACE_Y + cavity_depth
                and abs(center.Z - CONNECTOR_Z) < socket_height / 2 + 1e-6):
            face.color = COLOR_CREAMY_WHITE
            if abs(center.Z - CONNECTOR_Z) <= CONNECTOR_CONTACT_SIZE / 2 + 1e-6:
                if abs(center.X) < 2.1:
                    face.color = COLOR_BRASS
    return result


def main() -> None:
    result = build_model()
    show(result, name="lidar_ydlidar_tmini", clear=True)


if __name__ == "__main__":
    main()
