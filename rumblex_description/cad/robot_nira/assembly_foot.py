#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR

from build123d import Compound, Pos, Rot, export_step

FOOT_SPACER_OUTER_DIAMETER = 4.0
FOOT_SPACER_STUD_HOLE_DIAMETER = 2.5
import robot_nira.foot_back as foot_back
import robot_nira.foot_connection as foot_connection
import robot_nira.foot_front as foot_front
import utils.spacer as spacer
import common.toe as toe
import robot_nira.foot_outer as foot_outer
from robot_nira.foot_common import FOOT_SPACER_OVERALL_LENGTH
from utils.ocp_utils import show

from utils.colors import COLOR_CREAMY_WHITE, COLOR_WINE_RED, COLOR_DARK_GRAY


TARGET_HOLE_X = foot_back.FOOT_MOUNT_HOLES[0]["x"]
TARGET_HOLE_Y = foot_back.FOOT_MOUNT_HOLES[0]["y"]

def build_assembly() -> Compound:
    connection_part = foot_connection.build_model(foot_connection.build_surface())
    foot_front_part = foot_front.build_model(foot_front.build_surface())
    foot_back_part = foot_back.build_model(foot_back.build_surface())
    toe_part = toe.build_model()
    spacer_part = spacer.build_model(
        outer_diameter=FOOT_SPACER_OUTER_DIAMETER,
        inner_diameter=FOOT_SPACER_STUD_HOLE_DIAMETER,
        length=FOOT_SPACER_OVERALL_LENGTH,
    )

    foot_front_part.color = COLOR_CREAMY_WHITE
    foot_back_part.color = COLOR_CREAMY_WHITE
    connection_part.color = COLOR_CREAMY_WHITE
    toe_part.color = COLOR_DARK_GRAY
    spacer_part.color = COLOR_WINE_RED

    # Keep the connection plate's rectangular outline centered on the tibia axis.
    # This lets the top/bottom hole pattern match the connection outline geometry.
    connection_offset_x = TARGET_HOLE_X + FOOT_SPACER_OUTER_DIAMETER/2
    connection_offset_y = TARGET_HOLE_Y

    # Stack cheeks along +Z; the toe connection and outer armor bridge them.
    foot_front_part.label = "foot_front"
    foot_back_part.label = "foot_back"
    foot_back_placed = foot_back_part
    spacer_z = foot_back.THICKNESS + FOOT_SPACER_OVERALL_LENGTH / 2
    spacer_placed_parts = [
        Pos(hole["x"], hole["y"], spacer_z) * spacer_part
        for hole in foot_back.FOOT_MOUNT_HOLES
    ]
    foot_front_placed = Pos(0, 0, foot_back.THICKNESS + FOOT_SPACER_OVERALL_LENGTH) * foot_front_part
    
    connection_placed = Pos(
        connection_offset_x,
        connection_offset_y,
        foot_back.THICKNESS + FOOT_SPACER_OVERALL_LENGTH/2,
    ) * Rot(0, 90, 0) * connection_part

    toe_placed = Pos(
        connection_offset_x + toe.LITTLE_TOE_LENGTH + foot_connection.THICKNESS,
        connection_offset_y,
        foot_back.THICKNESS + FOOT_SPACER_OVERALL_LENGTH / 2,
    ) * Rot(0, 90, 0) * Pos(0, 0, -toe.LITTLE_TOE_LENGTH / 2) * toe_part

    return Compound(
        children=[
            connection_placed,
            toe_placed,
            foot_front_placed,
            foot_back_placed,
            *spacer_placed_parts,
            foot_outer.build_placed_model(),
        ]
    )


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / "step/assembly_foot.step"))

    show(assembly, name="assembly_foot", clear=True)


if __name__ == "__main__":
    main()
