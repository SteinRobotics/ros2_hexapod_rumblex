#!/usr/bin/env python3
"""Angular, vented laser-cut cover for Nira's lidar interface.

Each component is a constant-thickness flat plate. The walls interlock at
their corners and pass through slots in the lid and body deck.
All fits are nominal; compensate for material thickness and laser kerf.
"""

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    Align, Box, BuildPart, BuildSketch, Compound, Cylinder, ExportDXF, Locations,
    Mode, Part, Plane, Polygon, Pos, Vector, export_step, export_stl, extrude,
)

from robot_nira import EXPORT_DIR, dxf_directory, board_ydlidar_tmini_interface as interface
from robot_nira import lidar_ydlidar_tmini as lidar
import robot_nira.torso_common as torso_common
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show

WALL = 1.5
CLEARANCE = 1.0
HEIGHT = interface.TOTAL_HEIGHT + CLEARANCE + WALL + 3.0
# The housing rotates 180 degrees in the torso: local -X is robot front.
# Extend that end for the forward-facing lidar plug; keep deck tabs fixed.
HALF_X = interface.BASE_DEPTH / 2 + CLEARANCE + WALL
FRONT_EXTENSION = 4.0
MIN_X = -HALF_X - FRONT_EXTENSION
MAX_X = HALF_X
CENTER_X = (MIN_X + MAX_X) / 2
# Sensor rotation relative to the housing is +90 degrees about Z.
LIDAR_MOUNT_POSITIONS = tuple(
    (CENTER_X - y, x) for x, y in lidar.MOUNT_HOLE_POSITIONS
)
LIDAR_MOUNT_DIAMETER = lidar.MOUNT_HOLE_DIAMETER + 0.2
HALF_Y = interface.BASE_WIDTH / 2 + CLEARANCE + WALL
FINGER_HEIGHT = 2.0
LID_TAB_WIDTH = 4.0
SHOULDER = 3.0


def apportioned_tabs(start: float, end: float, count: int, width: float) -> tuple[float, ...]:
    """Place fingers with equal clear gaps, including the two end margins."""
    gap = (end - start - count * width) / (count + 1)
    return tuple(start + gap + width / 2 + i * (width + gap) for i in range(count))


CORNER_FINGER_Z = tuple(
    z - FINGER_HEIGHT / 2
    for z in apportioned_tabs(0, HEIGHT - WALL - SHOULDER, 2, FINGER_HEIGHT)
)
SIDE_LID_TAB_X = apportioned_tabs(MIN_X + WALL + 4, MAX_X - WALL - 4, 3, LID_TAB_WIDTH)
# Four fingers leave the center free for the cable notch.
END_LID_TAB_Y = apportioned_tabs(-HALF_Y + 4, HALF_Y - 4, 4, LID_TAB_WIDTH)
DECK_TAB_X = (-12.0, 0.0, 12.0)
DECK_TAB_WIDTH = 4.0
DECK_TAB_DEPTH = torso_common.THICKNESS
DECK_SLOT_Y = HALF_Y - WALL / 2
DECK_TAB_POSITIONS = tuple(
    (x, y) for x in DECK_TAB_X for y in (-DECK_SLOT_Y, DECK_SLOT_Y)
)
BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)
# Extra room around each receptacle for the USB plug nose.
USB_PLUG_CLEARANCE = 2.0
# Rear panel is local +X, with +Y to the viewer's right from outside.
# The board's X coordinates map to housing -Y after assembly rotations.
USB_CUTOUTS = tuple(
    (-x, interface.USB_Z, width + 2 * USB_PLUG_CLEARANCE,
     height + 2 * USB_PLUG_CLEARANCE)
    for _, x, width, height in interface.USB_PORTS
)
LIDAR_CABLE_SIZE = 3.0
LIDAR_CABLE_Y = 0.0
LIDAR_CABLE_Z = HEIGHT - WALL - LIDAR_CABLE_SIZE / 2


def wall_outline(half_width: float, bottom: float, center: float = 0.0) -> None:
    """Cut the high corners back while keeping the corner fingers below them."""
    with BuildSketch() as outline:
        Polygon(
            (center - half_width, bottom), (center + half_width, bottom),
            (center + half_width, HEIGHT - WALL - SHOULDER),
            (center + half_width - 4, HEIGHT - WALL),
            (center - half_width + 4, HEIGHT - WALL),
            (center - half_width, HEIGHT - WALL - SHOULDER), align=None,
        )
    extrude(outline.sketch, amount=WALL)

def side_wall(mirrored: bool = False) -> Part:
    """Flat pattern: horizontal coordinate is X, vertical coordinate is Z."""
    span = (MAX_X - MIN_X) / 2 - WALL
    center = (MIN_X + MAX_X) / 2 * (-1 if mirrored else 1)
    with BuildPart() as panel:
        wall_outline(span, 0, center)
        for end in (-1, 1):
            for z in CORNER_FINGER_Z:
                with Locations((center + end * (span + WALL / 2), z + FINGER_HEIGHT / 2, 0)):
                    Box(WALL, FINGER_HEIGHT, WALL, align=BOTTOM)
        for x in SIDE_LID_TAB_X:
            with Locations((-x if mirrored else x, HEIGHT - WALL / 2, 0)):
                Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM)
        for x in DECK_TAB_X:
            with Locations((x, (WALL - DECK_TAB_DEPTH) / 2, 0)):
                Box(DECK_TAB_WIDTH, WALL + DECK_TAB_DEPTH, WALL, align=BOTTOM)
    return panel.part


def end_wall(front: bool) -> Part:
    """Flat pattern with side fingers and three rear connector cutouts."""
    with BuildPart() as panel:
        wall_outline(HALF_Y, 0)
        for end in (-1, 1):
            for z in CORNER_FINGER_Z:
                with Locations((end * (HALF_Y - WALL / 2), z + FINGER_HEIGHT / 2, 0)):
                    Box(WALL, FINGER_HEIGHT, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
        for y in END_LID_TAB_Y:
            with Locations((y, HEIGHT - WALL / 2, 0)):
                Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM)
        if front:
            for y, z, width, height in USB_CUTOUTS:
                with Locations((y, z, 0)):
                    Box(width, height, WALL,
                        align=BOTTOM, mode=Mode.SUBTRACT)
            with Locations((LIDAR_CABLE_Y, LIDAR_CABLE_Z, 0)):
                Box(LIDAR_CABLE_SIZE, LIDAR_CABLE_SIZE, WALL,
                    align=BOTTOM, mode=Mode.SUBTRACT)
    return panel.part


def lid() -> Part:
    with BuildPart() as panel:
        # The clipped corners and paired swept openings repeat the outer foot.
        with BuildSketch() as surface:
            Polygon(
                (MIN_X + 4, -HALF_Y), (MAX_X - 4, -HALF_Y),
                (MAX_X, -HALF_Y + 4), (MAX_X, HALF_Y - 4),
                (MAX_X - 4, HALF_Y), (MIN_X + 4, HALF_Y),
                (MIN_X, HALF_Y - 4), (MIN_X, -HALF_Y + 4),
                align=None,
            )

        extrude(amount=WALL)
        with Locations(*LIDAR_MOUNT_POSITIONS):
            Cylinder(LIDAR_MOUNT_DIAMETER / 2, WALL,
                     align=BOTTOM, mode=Mode.SUBTRACT)
        for y in (-HALF_Y + WALL / 2, HALF_Y - WALL / 2):
            for x in SIDE_LID_TAB_X:
                with Locations((x, y, 0)):
                    Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
        for x in (MIN_X + WALL / 2, MAX_X - WALL / 2):
            for y in END_LID_TAB_Y:
                with Locations((x, y, 0)):
                    Box(WALL, LID_TAB_WIDTH, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
    return panel.part


def flat_parts() -> dict[str, Part]:
    """Return one cut-ready flat solid per physical piece."""
    return {
        "lid": lid(),
        "side_negative_y": side_wall(),
        "side_positive_y": side_wall(mirrored=True),
        "front": end_wall(True),
        "back": end_wall(False),
    }


def assemble(panels: dict[str, Part]) -> Part:
    side_negative = Plane(
        origin=(0, -HALF_Y + WALL, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)
    )
    side_positive = Plane(
        origin=(0, HALF_Y - WALL, 0), x_dir=(-1, 0, 0), z_dir=(0, 1, 0)
    )
    front = Plane(
        origin=(HALF_X - WALL, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)
    )
    back = Plane(
        origin=(MIN_X + WALL, 0, 0), x_dir=(0, -1, 0), z_dir=(-1, 0, 0)
    )
    placements = {
        "lid": Pos(0, 0, HEIGHT - WALL),
        "side_negative_y": side_negative.location,
        "side_positive_y": side_positive.location,
        "front": front.location,
        "back": back.location,
    }
    pieces = []
    for name, panel in panels.items():
        piece = placements[name] * panel
        piece.label = name
        piece.color = COLOR_CREAMY_WHITE
        pieces.append(piece)
    # A flat OCC compound keeps all five solids in one place when the robot
    # assembly is moved. A nested child tree loses its parent placement during
    # boolean checks in build123d.
    result = Part(Compound(pieces).wrapped, label="lidar_interface_housing")
    result.color = COLOR_CREAMY_WHITE
    return result


def build_model() -> Part:
    return assemble(flat_parts())


def export_flat_parts(panels: dict[str, Part]) -> None:
    """Write the actual five cut patterns, including every tab and slot."""
    dxf_dir = dxf_directory(WALL) / "lidar_interface_housing"
    dxf_dir.mkdir(parents=True, exist_ok=True)
    for name, panel in panels.items():
        top = [face for face in panel.faces()
               if face.normal_at().dot(Vector(0, 0, 1)) > 1 - 1e-6
               and abs(face.center().Z - WALL) < 1e-6]
        if len(top) != 1:
            raise ValueError(f"Expected one top face for {name}, found {len(top)}")
        drawing = ExportDXF()
        drawing.add_shape(Plane(origin=(0, 0, WALL)).to_local_coords(top[0]))
        drawing.write(str(dxf_dir / f"{name}.dxf"))


def main() -> None:
    panels = flat_parts()
    part = assemble(panels)
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_flat_parts(panels)
    show(part, name=part.label, clear=True)


if __name__ == "__main__":
    main()
