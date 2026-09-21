#!/usr/bin/env python3
"""Photo-based YDLIDAR T-mini interface enclosure, in millimetres.

Models the separate low black enclosure with side mounting flanges and a
cream connector in the supplied product photo, not the rotating lidar.
ALL dimensions are estimates: no dimensioned interface-board drawing was
provided. Verify the envelope, hole pattern and connector against hardware
before designing mating parts. Internal electronics, cables, lettering and
unseen ports are omitted; the connector contacts are illustrative.

X spans the mounting flanges, the connector faces -Y, and the mounting face
is Z=0. The mount and mount_1..4 joints lie on that face.
"""

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    Align, Box, BuildPart, BuildSketch, Compound, Cylinder, Location,
    Locations, Mode, Plane, RectangleRounded, RigidJoint,
    export_step, export_stl, extrude, loft,
)

from robot_nira import EXPORT_DIR
from utils.colors import COLOR_BRASS, COLOR_CREAMY_WHITE, COLOR_DARK_GRAY
from utils.ocp_utils import show

# Photo-derived placeholders, not manufacturer specifications.
BASE_WIDTH = 48.0
BASE_DEPTH = 40.0
BASE_THICKNESS = 2.0
BASE_CORNER_RADIUS = 2.0
BODY_WIDTH = 36.0
BODY_DEPTH = 38.0
TOTAL_HEIGHT = 10.0
COVER_TAPER = 1.0
WALL_THICKNESS = 1.2
MOUNT_HOLE_SPACING_X = 42.0
MOUNT_HOLE_SPACING_Y = 28.0
MOUNT_HOLE_DIAMETER = 3.2
CONNECTOR_WIDTH = 10.0
CONNECTOR_DEPTH = 5.0
CONNECTOR_HEIGHT = 4.0
CONNECTOR_RECESS = 0.8
CONNECTOR_WALL = 0.7
CONTACT_COUNT = 4
CONTACT_PITCH = 2.0
CONTACT_SIZE = 0.45
MOUNT_HOLE_POSITIONS = tuple(
    (x, y)
    for x in (-MOUNT_HOLE_SPACING_X / 2, MOUNT_HOLE_SPACING_X / 2)
    for y in (-MOUNT_HOLE_SPACING_Y / 2, MOUNT_HOLE_SPACING_Y / 2)
)
BOTTOM_ALIGN = (Align.CENTER, Align.CENTER, Align.MIN)


def build_model() -> Compound:
    """Return the enclosure, recessed socket and illustrative contacts."""
    connector_front = -BODY_DEPTH / 2 + CONNECTOR_RECESS
    connector_y = connector_front + CONNECTOR_DEPTH / 2
    connector_z = BASE_THICKNESS + CONNECTOR_HEIGHT / 2

    with BuildPart() as housing:
        with BuildSketch():
            RectangleRounded(BASE_WIDTH, BASE_DEPTH, BASE_CORNER_RADIUS)
        extrude(amount=BASE_THICKNESS)

        # Gently tapered cover on a flat flange, as seen in the reference.
        with BuildSketch(Plane.XY.offset(BASE_THICKNESS)):
            RectangleRounded(BODY_WIDTH, BODY_DEPTH, 1.5)
        with BuildSketch(Plane.XY.offset(TOTAL_HEIGHT)):
            RectangleRounded(
                BODY_WIDTH - 2 * COVER_TAPER,
                BODY_DEPTH - 2 * COVER_TAPER, 1.5,
            )
        loft()
        with Locations((0, 0, BASE_THICKNESS)):
            Box(
                BODY_WIDTH - 2 * (WALL_THICKNESS + COVER_TAPER),
                BODY_DEPTH - 2 * (WALL_THICKNESS + COVER_TAPER),
                TOTAL_HEIGHT - BASE_THICKNESS - WALL_THICKNESS,
                align=BOTTOM_ALIGN, mode=Mode.SUBTRACT,
            )
        with Locations(*MOUNT_HOLE_POSITIONS):
            Cylinder(
                MOUNT_HOLE_DIAMETER / 2, BASE_THICKNESS,
                align=BOTTOM_ALIGN, mode=Mode.SUBTRACT,
            )
        # Opening through the front wall; the socket sits behind the lip.
        with Locations((0, -BODY_DEPTH / 2, connector_z)):
            Box(
                CONNECTOR_WIDTH + 1.2, 2 * (CONNECTOR_DEPTH + COVER_TAPER),
                CONNECTOR_HEIGHT + 0.8, mode=Mode.SUBTRACT,
            )
    housing.part.label = "interface_enclosure"
    housing.part.color = COLOR_DARK_GRAY

    with BuildPart() as socket:
        with Locations((0, connector_y, connector_z)):
            Box(CONNECTOR_WIDTH, CONNECTOR_DEPTH, CONNECTOR_HEIGHT)
        with Locations((0, connector_y - CONNECTOR_WALL / 2, connector_z)):
            Box(
                CONNECTOR_WIDTH - 2 * CONNECTOR_WALL,
                CONNECTOR_DEPTH - CONNECTOR_WALL,
                CONNECTOR_HEIGHT - 2 * CONNECTOR_WALL,
                mode=Mode.SUBTRACT,
            )
    socket.part.label = "interface_socket"
    socket.part.color = COLOR_CREAMY_WHITE

    contacts = []
    for index in range(CONTACT_COUNT):
        x = (index - (CONTACT_COUNT - 1) / 2) * CONTACT_PITCH
        with BuildPart() as contact:
            with Locations((x, connector_y + 0.4, connector_z)):
                Box(CONTACT_SIZE, CONNECTOR_DEPTH - 0.8, CONTACT_SIZE)
        contact.part.label = f"socket_contact_{index + 1}"
        contact.part.color = COLOR_BRASS
        contacts.append(contact.part)

    result = Compound(
        label="board_ydlidar_tmini_interface",
        children=[housing.part, socket.part, *contacts],
    )
    RigidJoint("mount", to_part=result, joint_location=Location((0, 0, 0)))
    for index, (x, y) in enumerate(MOUNT_HOLE_POSITIONS, start=1):
        RigidJoint(
            f"mount_{index}", to_part=result,
            joint_location=Location((x, y, 0)),
        )
    RigidJoint(
        "connector", to_part=result,
        joint_location=Location(Plane(
            origin=(0, connector_front, connector_z),
            x_dir=(1, 0, 0), z_dir=(0, -1, 0),
        )),
    )
    return result


def main() -> None:
    result = build_model()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "board_ydlidar_tmini_interface.step"))
    export_stl(result, str(EXPORT_DIR / "board_ydlidar_tmini_interface.stl"))
    show(result, name="board_ydlidar_tmini_interface", clear=True)


if __name__ == "__main__":
    main()
