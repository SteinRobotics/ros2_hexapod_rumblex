#!/usr/bin/env python3
"""Laser-cut lidar interface housing with an open rear USB tunnel.

Four constant-thickness plates interlock at their corners and pass through
slots in the lid and body deck. Two foot-style spacers fasten the lid to the deck.
All fits are nominal; compensate for material thickness and laser kerf.
"""

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    Align, Box, BuildPart, BuildSketch, Compound, Cylinder, ExportDXF, Locations,
    Mode, Part, Plane, Polygon, Pos, Rectangle, Vector, insert, extrude,
)

from robot_nira import EXPORT_DIR, dxf_directory, board_ydlidar_tmini_interface as interface
from robot_nira import lidar_ydlidar_tmini as lidar
import robot_nira.torso_common as torso_common
from robot_nira.chassis_common import (
    RAIL_HEIGHT, SLOPE_COVER_SETBACK, SLOPE_END_X, SLOPE_START_X,
)
from robot_nira.lidar_layout import LIDAR_ASSEMBLY_X, LOW_RAIL_HEIGHT
from utils import spacer
from utils.colors import COLOR_CREAMY_WHITE, COLOR_DARK_GRAY, COLOR_TITAN_SILVER, COLOR_WINE_RED
from utils.ocp_utils import show

WALL = 1.5
CLEARANCE = 1.0
HEIGHT = interface.TOTAL_HEIGHT + CLEARANCE + WALL + 3.0
# The housing rotates 180 degrees in the torso: local -X is robot front.
# Extend that end for the forward-facing lidar plug; keep deck tabs fixed.
HALF_X = interface.BASE_DEPTH / 2 + CLEARANCE + WALL
FRONT_EXTENSION = 4.0
MIN_X = -HALF_X - FRONT_EXTENSION
# Keep the scanner over the original housing, independent of tunnel length.
CENTER_X = (MIN_X + HALF_X) / 2
# Reach the outside face of the slope at the tunnel roof. Local +X is rearward.
SLOPE_RUN = SLOPE_END_X - SLOPE_START_X
SLOPE_DROP = RAIL_HEIGHT - LOW_RAIL_HEIGHT
TUNNEL_END_X = (
    SLOPE_END_X - SLOPE_COVER_SETBACK
    - (HEIGHT - LOW_RAIL_HEIGHT) * SLOPE_RUN / SLOPE_DROP
)
MAX_X = LIDAR_ASSEMBLY_X - TUNNEL_END_X
# Sensor rotation relative to the housing is +90 degrees about Z.
LIDAR_MOUNT_POSITIONS = tuple(
    (CENTER_X - y, x) for x, y in lidar.MOUNT_HOLE_POSITIONS
)
LIDAR_MOUNT_DIAMETER = lidar.MOUNT_HOLE_DIAMETER + 0.2
HALF_Y = interface.BASE_WIDTH / 2 + CLEARANCE + WALL
FINGER_HEIGHT = 2.0
LID_TAB_WIDTH = 4.0
# Material ahead of the front-panel fingers closes the side-panel slots.
CORNER_RIM = 3.0
SIDE_MIN_X = MIN_X - CORNER_RIM
# Reuse the foot's spacer stock outside the walls, leaving the electronics
# and the entire rear cable passage clear.
ROOF_RIM = 3.0
SPACER_OUTER_DIAMETER = spacer.FOOT_SPACER_OUTER_DIAMETER
SPACER_INNER_DIAMETER = spacer.FOOT_SPACER_STUD_HOLE_DIAMETER
SPACER_LENGTH = HEIGHT - WALL
SPACER_SIDE_CLEARANCE = 0.0
SPACER_SIDE_Y = HALF_Y + SPACER_OUTER_DIAMETER / 2 + SPACER_SIDE_CLEARANCE
ROOF_MIN_X = SIDE_MIN_X - ROOF_RIM
ROOF_HALF_Y = SPACER_SIDE_Y + SPACER_OUTER_DIAMETER / 2 + 2.0
# Clip back to the original tunnel width before reaching the slope cover.
ROOF_CHAMFER = ROOF_HALF_Y - HALF_Y + ROOF_RIM


def apportioned_tabs(start: float, end: float, count: int, width: float) -> tuple[float, ...]:
    """Place fingers with equal clear gaps, including the two end margins."""
    gap = (end - start - count * width) / (count + 1)
    return tuple(start + gap + width / 2 + i * (width + gap) for i in range(count))


CORNER_FINGER_Z = tuple(
    z - FINGER_HEIGHT / 2
    for z in apportioned_tabs(0, HEIGHT - WALL, 2, FINGER_HEIGHT)
)
SIDE_LID_TAB_X = apportioned_tabs(
    SIDE_MIN_X + ROOF_RIM, MAX_X - ROOF_CHAMFER, 3, LID_TAB_WIDTH)
# Center both supports and their lid/deck bores on the middle roof finger.
SPACER_POSITIONS = tuple(
    (SIDE_LID_TAB_X[len(SIDE_LID_TAB_X) // 2], sign * SPACER_SIDE_Y)
    for sign in (-1, 1)
)
END_LID_TAB_Y = apportioned_tabs(
    -HALF_Y + ROOF_CHAMFER, HALF_Y - ROOF_CHAMFER, 3, LID_TAB_WIDTH)
DECK_TAB_X = (-12.0, 0.0, 12.0)
DECK_TAB_WIDTH = 4.0
DECK_TAB_DEPTH = torso_common.THICKNESS
DECK_SLOT_Y = HALF_Y - WALL / 2
DECK_TAB_POSITIONS = tuple(
    (x, y) for x in DECK_TAB_X for y in (-DECK_SLOT_Y, DECK_SLOT_Y)
)
BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)


def side_wall(mirrored: bool = False) -> Part:
    """Flat pattern: horizontal coordinate is X, vertical coordinate is Z."""
    sign = -1 if mirrored else 1
    # Follow the slope at the open end, keeping the speaker space behind it
    # clear. Below the cover's lower edge, stop at that edge's X coordinate.
    rear_bottom = LIDAR_ASSEMBLY_X - SLOPE_END_X + SLOPE_COVER_SETBACK
    rear_top = MAX_X - WALL * SLOPE_RUN / SLOPE_DROP
    with BuildPart() as panel:
        with BuildSketch():
            Polygon(*[(sign * x, z) for x, z in (
                (SIDE_MIN_X, 0), (rear_bottom, 0),
                (rear_bottom, LOW_RAIL_HEIGHT),
                (rear_top, HEIGHT - WALL),
                (SIDE_MIN_X, HEIGHT - WALL),
            )], align=None)
        extrude(amount=WALL)
        for z in CORNER_FINGER_Z:
            with Locations((sign * (MIN_X + WALL / 2), z + FINGER_HEIGHT / 2, 0)):
                Box(WALL, FINGER_HEIGHT, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
        for x in SIDE_LID_TAB_X:
            with Locations((-x if mirrored else x, HEIGHT - WALL / 2, 0)):
                Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM)
        for x in DECK_TAB_X:
            with Locations((x, (WALL - DECK_TAB_DEPTH) / 2, 0)):
                Box(DECK_TAB_WIDTH, WALL + DECK_TAB_DEPTH, WALL, align=BOTTOM)
    return panel.part


def end_wall() -> Part:
    """Robot-front panel; the rear end stays open for USB cables."""
    with BuildPart() as panel:
        with Locations((0, (HEIGHT - WALL) / 2, 0)):
            Box(2 * (HALF_Y - WALL), HEIGHT - WALL, WALL, align=BOTTOM)
        for end in (-1, 1):
            for z in CORNER_FINGER_Z:
                with Locations((end * (HALF_Y - WALL / 2), z + FINGER_HEIGHT / 2, 0)):
                    Box(WALL, FINGER_HEIGHT, WALL, align=BOTTOM)
        for y in END_LID_TAB_Y:
            with Locations((y, HEIGHT - WALL / 2, 0)):
                Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM)
    return panel.part


def lid() -> Part:
    with BuildPart() as panel:
        # Reuse the body's octagonal plate layout, with a square USB mouth.
        outline = torso_common.chamfered_octagon(
            MAX_X - ROOF_MIN_X, 2 * ROOF_HALF_Y, ROOF_CHAMFER)
        with BuildSketch():
            with Locations(((ROOF_MIN_X + MAX_X) / 2, 0)):
                insert(outline)
            with Locations((MAX_X - ROOF_CHAMFER / 2, 0)):
                Rectangle(ROOF_CHAMFER, 2 * HALF_Y)
        extrude(amount=WALL)
        with Locations(*LIDAR_MOUNT_POSITIONS):
            Cylinder(LIDAR_MOUNT_DIAMETER / 2, WALL,
                     align=BOTTOM, mode=Mode.SUBTRACT)
        with Locations(*SPACER_POSITIONS):
            Cylinder(SPACER_INNER_DIAMETER / 2, WALL,
                     align=BOTTOM, mode=Mode.SUBTRACT)
        for y in (-HALF_Y + WALL / 2, HALF_Y - WALL / 2):
            for x in SIDE_LID_TAB_X:
                with Locations((x, y, 0)):
                    Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
        for y in END_LID_TAB_Y:
            with Locations((MIN_X + WALL / 2, y, 0)):
                Box(WALL, LID_TAB_WIDTH, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
    return panel.part


def roof_spacers() -> list[Part]:
    """Two reusable spacer connections, seated between deck and roof underside."""
    stock = spacer.build_model(
        outer_diameter=SPACER_OUTER_DIAMETER,
        inner_diameter=SPACER_INNER_DIAMETER,
        length=SPACER_LENGTH,
    )
    posts = []
    for index, (x, y) in enumerate(SPACER_POSITIONS, start=1):
        post = Pos(x, y, SPACER_LENGTH / 2) * stock
        post.label = f"lidar_roof_spacer_{index}"
        post.color = COLOR_WINE_RED
        posts.append(post)
    return posts


def flat_parts() -> dict[str, Part]:
    """Return one cut-ready flat solid per physical piece."""
    return {
        "lid": lid(),
        "side_negative_y": side_wall(),
        "side_positive_y": side_wall(mirrored=True),
        # Retain the original flat-pattern name for the robot-front panel.
        "back": end_wall(),
    }


def assemble(panels: dict[str, Part]) -> Compound:
    side_negative = Plane(
        origin=(0, -HALF_Y + WALL, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)
    )
    side_positive = Plane(
        origin=(0, HALF_Y - WALL, 0), x_dir=(-1, 0, 0), z_dir=(0, 1, 0)
    )
    back = Plane(
        origin=(MIN_X + WALL, 0, 0), x_dir=(0, -1, 0), z_dir=(-1, 0, 0)
    )
    placements = {
        "lid": Pos(0, 0, HEIGHT - WALL),
        "side_negative_y": side_negative.location,
        "side_positive_y": side_positive.location,
        "back": back.location,
    }
    pieces = []
    for name, panel in panels.items():
        piece = placements[name] * panel
        piece.label = name
        piece.color = COLOR_DARK_GRAY
        pieces.append(piece)
    pieces.extend(roof_spacers())
    # Retain the individual parts so the wine-red spacers keep their colors
    # in the viewer and STEP export, as they do in the foot assembly.
    return Compound(children=pieces, label="lidar_interface_housing")


def build_model() -> Compound:
    return assemble(flat_parts())


def export_flat_parts(panels: dict[str, Part]) -> None:
    """Write the four cut patterns, including every tab and slot."""
    dxf_dir = dxf_directory(WALL) / "lidar_interface_housing"
    dxf_dir.mkdir(parents=True, exist_ok=True)
    # This was the removed robot-rear panel in the original housing.
    (dxf_dir / "front.dxf").unlink(missing_ok=True)
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
