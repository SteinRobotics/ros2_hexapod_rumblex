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
    Align, Box, BuildPart, BuildSketch, Compound, ExportDXF, Locations,
    Mode, Part, Plane, Polygon, Pos, Vector, export_step, export_stl, extrude,
)

from robot_nira import EXPORT_DIR, board_ydlidar_tmini_interface as interface
import robot_nira.body_common as body_common
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show

WALL = 1.5
CLEARANCE = 1.0
HEIGHT = interface.TOTAL_HEIGHT + CLEARANCE + WALL
# In this housing's local frame, the board is rotated 90 degrees and its socket faces +X.
# The body assembly rotates both parts so the socket and cable opening face inward.
HALF_X = interface.BASE_DEPTH / 2 + CLEARANCE + WALL
HALF_Y = interface.BASE_WIDTH / 2 + CLEARANCE + WALL
FINGER_HEIGHT = 2.0
CORNER_FINGER_Z = (2.0, 6.0)
LID_TAB_WIDTH = 4.0
SIDE_LID_TAB_X = (-12.0, 0.0, 12.0)
END_LID_TAB_Y = (-14.0, 0.0, 14.0)
DECK_TAB_X = (-12.0, 0.0, 12.0)
DECK_TAB_WIDTH = 4.0
DECK_TAB_DEPTH = body_common.THICKNESS
DECK_SLOT_Y = HALF_Y - WALL / 2
DECK_TAB_POSITIONS = tuple(
    (x, y) for x in DECK_TAB_X for y in (-DECK_SLOT_Y, DECK_SLOT_Y)
)
BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)
SHOULDER = 3.0


def wall_outline(half_width: float, bottom: float) -> None:
    """Cut the high corners back while keeping the corner fingers below them."""
    with BuildSketch() as outline:
        Polygon(
            (-half_width, bottom), (half_width, bottom),
            (half_width, HEIGHT - WALL - SHOULDER),
            (half_width - 4, HEIGHT - WALL),
            (-half_width + 4, HEIGHT - WALL),
            (-half_width, HEIGHT - WALL - SHOULDER), align=None,
        )
    extrude(outline.sketch, amount=WALL)

def side_wall() -> Part:
    """Flat pattern: horizontal coordinate is X, vertical coordinate is Z."""
    span = HALF_X - WALL
    with BuildPart() as panel:
        wall_outline(span, 0)
        for end in (-1, 1):
            for z in CORNER_FINGER_Z:
                with Locations((end * (span + WALL / 2), z + FINGER_HEIGHT / 2, 0)):
                    Box(WALL, FINGER_HEIGHT, WALL, align=BOTTOM)
        for x in SIDE_LID_TAB_X:
            with Locations((x, HEIGHT - WALL / 2, 0)):
                Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM)
        for x in DECK_TAB_X:
            with Locations((x, (WALL - DECK_TAB_DEPTH) / 2, 0)):
                Box(DECK_TAB_WIDTH, WALL + DECK_TAB_DEPTH, WALL, align=BOTTOM)
    return panel.part


def end_wall(front: bool) -> Part:
    """Flat pattern with side-finger recesses and a bottom cable opening."""
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
            with Locations((0, 4, 0)):
                Box(interface.CONNECTOR_WIDTH + 4, 8, WALL,
                    align=BOTTOM, mode=Mode.SUBTRACT)
    return panel.part


def lid() -> Part:
    with BuildPart() as panel:
        # The clipped corners and paired swept openings repeat the outer foot.
        with BuildSketch() as surface:
            Polygon(
                (-HALF_X + 4, -HALF_Y), (HALF_X - 4, -HALF_Y),
                (HALF_X, -HALF_Y + 4), (HALF_X, HALF_Y - 4),
                (HALF_X - 4, HALF_Y), (-HALF_X + 4, HALF_Y),
                (-HALF_X, HALF_Y - 4), (-HALF_X, -HALF_Y + 4),
                align=None,
            )

        extrude(amount=WALL)
        for y in (-HALF_Y + WALL / 2, HALF_Y - WALL / 2):
            for x in SIDE_LID_TAB_X:
                with Locations((x, y, 0)):
                    Box(LID_TAB_WIDTH, WALL, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
        for x in (-HALF_X + WALL / 2, HALF_X - WALL / 2):
            for y in END_LID_TAB_Y:
                with Locations((x, y, 0)):
                    Box(WALL, LID_TAB_WIDTH, WALL, align=BOTTOM, mode=Mode.SUBTRACT)
    return panel.part


def flat_parts() -> dict[str, Part]:
    """Return one cut-ready flat solid per physical piece."""
    return {
        "lid": lid(),
        "side_negative_y": side_wall(),
        "side_positive_y": side_wall(),
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
        origin=(-HALF_X + WALL, 0, 0), x_dir=(0, -1, 0), z_dir=(-1, 0, 0)
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
    dxf_dir = EXPORT_DIR / "dxf" / "lidar_interface_housing"
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
    (EXPORT_DIR / "stl").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    export_step(part, str(EXPORT_DIR / "step/lidar_interface_housing.step"))
    export_stl(part, str(EXPORT_DIR / "stl/lidar_interface_housing.stl"))
    export_flat_parts(panels)
    show(part, name=part.label, clear=True)


if __name__ == "__main__":
    main()
