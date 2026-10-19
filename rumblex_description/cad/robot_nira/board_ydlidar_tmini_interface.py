#!/usr/bin/env python3
"""Photo-based YDLIDAR T-mini interface enclosure, in millimetres.

Models the separate low black enclosure with side mounting flanges and a
cream connector in the supplied product photo, not the rotating lidar.
ALL dimensions are estimates: no dimensioned interface-board drawing was
provided. Verify the envelope, hole pattern and connector against hardware
before designing mating parts. Internal electronics, cables and lettering
are omitted; the connector contacts and USB tongues are illustrative.

Port types follow the YDLIDAR T-mini Plus User Manual V1.0 (231222),
section 1.1: USB_DATA is Type-C and USB_PWR is auxiliary Micro USB.
Reference (manufacturer manual, distributor mirror):
https://akizukidenshi.com/goodsaffix/YDLIDAR%20T-mini%20Plus%20User%20Manual_V1.0%20%28231222%29%20.pdf

X spans the mounting flanges, the lidar connector faces -Y, USB faces +Y,
and the mounting face
is Z=0. The mount and mount_1..4 joints lie on that face.
"""

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    Align, Box, BuildPart, BuildSketch, Compound, Cylinder, Location,
    Locations, Mode, Plane, Polygon, RectangleRounded, RigidJoint,
    export_step, export_stl, extrude, loft,
)

from robot_nira import EXPORT_DIR
from utils.colors import (
    COLOR_BRASS, COLOR_CREAMY_WHITE, COLOR_DARK_GRAY, COLOR_TITAN_SILVER,
)
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
# Estimated receptacle envelopes/placement, not manufacturer board dimensions.
USB_DEPTH = 6.0
USB_WALL = 0.3
USB_Z = 4.5
USB_FRONT = BODY_DEPTH / 2 - CONNECTOR_RECESS
USB_PORTS = (
    ("usb_data", 7.0, 9.0, 3.4),
    ("usb_pwr", -7.0, 8.0, 3.0),
)
MOUNT_HOLE_POSITIONS = tuple(
    (x, y)
    for x in (-MOUNT_HOLE_SPACING_X / 2, MOUNT_HOLE_SPACING_X / 2)
    for y in (-MOUNT_HOLE_SPACING_Y / 2, MOUNT_HOLE_SPACING_Y / 2)
)
BOTTOM_ALIGN = (Align.CENTER, Align.CENTER, Align.MIN)


def build_model() -> Compound:
    """Return the enclosure with lidar, USB-C data and Micro-USB power sockets."""
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
        for _, x, width, height in USB_PORTS:
            with Locations((x, BODY_DEPTH / 2, USB_Z)):
                Box(width + 0.8, 2 * USB_DEPTH, height + 0.8,
                    mode=Mode.SUBTRACT)
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

    usb_parts = []
    for name, x, width, height in USB_PORTS:
        # Local sketch Y is global Z; extrusion goes inward from the +Y face.
        face = Plane(origin=(x, USB_FRONT, USB_Z),
                     x_dir=(1, 0, 0), z_dir=(0, -1, 0))
        with BuildPart() as shell:
            with BuildSketch(face):
                if name == "usb_data":
                    RectangleRounded(width, height, height / 2 - 0.01)
                    RectangleRounded(width - 2 * USB_WALL,
                                     height - 2 * USB_WALL,
                                     height / 2 - USB_WALL - 0.01,
                                     mode=Mode.SUBTRACT)
                else:
                    # Micro-B's keyed, bevelled lower corners.
                    Polygon((-width / 2, height / 2), (width / 2, height / 2),
                            (width / 2, -height / 2 + 1),
                            (width / 2 - 1, -height / 2),
                            (-width / 2 + 1, -height / 2),
                            (-width / 2, -height / 2 + 1))
                    inner_w = width / 2 - USB_WALL
                    inner_h = height / 2 - USB_WALL
                    Polygon((-inner_w, inner_h), (inner_w, inner_h),
                            (inner_w, -inner_h + 0.85),
                            (inner_w - 0.85, -inner_h),
                            (-inner_w + 0.85, -inner_h),
                            (-inner_w, -inner_h + 0.85), mode=Mode.SUBTRACT)
            extrude(amount=USB_DEPTH)
        shell.part.label = f"{name}_shell"
        shell.part.color = COLOR_TITAN_SILVER
        with BuildPart() as tongue:
            with Locations((x, USB_FRONT - USB_DEPTH / 2 - 0.5, USB_Z)):
                Box(width - 2.5, USB_DEPTH - 1, 0.65)
        tongue.part.label = f"{name}_tongue"
        tongue.part.color = COLOR_DARK_GRAY
        usb_parts.extend((shell.part, tongue.part))

    result = Compound(
        label="board_ydlidar_tmini_interface",
        children=[housing.part, socket.part, *contacts, *usb_parts],
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
    for name, x, _, _ in USB_PORTS:
        RigidJoint(name, to_part=result, joint_location=Location(Plane(
            origin=(x, USB_FRONT, USB_Z), x_dir=(1, 0, 0), z_dir=(0, 1, 0),
        )))
    return result


def main() -> None:
    result = build_model()
    show(result, name="board_ydlidar_tmini_interface", clear=True)


if __name__ == "__main__":
    main()
