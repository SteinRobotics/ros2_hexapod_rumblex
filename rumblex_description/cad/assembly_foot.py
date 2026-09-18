#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step

FOOT_SPACER_OUTER_DIAMETER = 4.0
FOOT_SPACER_OVERALL_LENGTH = 32.0
FOOT_SPACER_STUD_HOLE_DIAMETER = 2.5
import foot_back
import foot_connection
import foot_front
import utils.spacer as spacer
import toe
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

    # Stack in +Z: foot_back -> spacer -> foot_front -> foot_connection.
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
        ]
    )


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_foot.step")

    show(assembly, name="assembly_foot", clear=True)


if __name__ == "__main__":
    main()
