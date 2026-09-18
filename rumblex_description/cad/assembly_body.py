#!/usr/bin/env python3

from pathlib import Path

from build123d import Color, Compound, Pos, Rot, export_step
from utils.ocp_utils import show

from utils.colors import COLOR_CREAMY_WHITE, COLOR_WINE_RED, COLOR_DARK_GRAY

import toe
import utils.spacer as spacer
import body_common
import body_layer_0
import body_layer_1
import body_layer_2

SPACER_OUTER_DIAMETER = 5.0
SPACER_INNER_DIAMETER = 3.0
SPACER_LENGTH_0_to_1 = 15.0
SPACER_LENGTH_1_to_2 = 32.0
SPACER_LENGTH_TOP = 45.0  


def build_assembly() -> Compound:
    body_layer_0_part = body_layer_0.build_model(body_layer_0.build_surface())
    body_layer_1_part = body_layer_1.build_model(body_layer_1.build_surface())
    body_layer_2_part = body_layer_2.build_model(body_layer_2.build_surface())
    toe_part = toe.build_model()

    spacer_part_0_1 = spacer.build_model(
        outer_diameter=SPACER_OUTER_DIAMETER,
        inner_diameter=SPACER_INNER_DIAMETER,
        length=SPACER_LENGTH_0_to_1,
    )
    spacer_part_1_2 = spacer.build_model(
        outer_diameter=SPACER_OUTER_DIAMETER,
        inner_diameter=SPACER_INNER_DIAMETER,
        length=SPACER_LENGTH_1_to_2,
    )
    spacer_part_top = spacer.build_model(
        outer_diameter=SPACER_OUTER_DIAMETER,
        inner_diameter=SPACER_INNER_DIAMETER,
        length=SPACER_LENGTH_TOP,
    )

    body_layer_0_part.color = COLOR_CREAMY_WHITE
    body_layer_1_part.color = COLOR_CREAMY_WHITE
    body_layer_2_part.color = COLOR_CREAMY_WHITE
    toe_part.color = COLOR_DARK_GRAY
    spacer_part_0_1.color = COLOR_WINE_RED
    spacer_part_1_2.color = COLOR_WINE_RED
    spacer_part_top.color = COLOR_WINE_RED

    body_layer_0_part.label = "body_layer_0"
    body_layer_1_part.label = "body_layer_1"
    body_layer_2_part.label = "body_layer_2"
    toe_part.label = "toe"

    positions_for_toes = body_layer_0.TOE_MOUNTING_HOLES
    positions_for_spacers = body_common.hole_locations

    # --- Z-stack the layers, separated by spacer lengths ---
    z_layer_0 = 0.0
    z_spacer_0_1 = z_layer_0 + body_common.THICKNESS
    z_layer_1 = z_spacer_0_1 + SPACER_LENGTH_0_to_1
    z_spacer_1_2 = z_layer_1 + body_common.THICKNESS
    z_layer_2 = z_spacer_1_2 + SPACER_LENGTH_1_to_2
    z_spacer_top = z_layer_2 + body_common.THICKNESS

    body_layer_1_part = Pos(0, 0, z_layer_1) * body_layer_1_part
    body_layer_1_part.label = "body_layer_1"
    body_layer_2_part = Pos(0, 0, z_layer_2) * body_layer_2_part
    body_layer_2_part.label = "body_layer_2"

    # Toes at the bottom, using their given (x, y, z) positions as-is
    toe_instances = []
    for i, coord in enumerate(positions_for_toes):
        x, y, z = coord
        instance = Pos(x, y, z - toe.LITTLE_TOE_LENGTH) * Rot(X=180) * toe_part
        instance.label = f"toe_{i}"
        toe_instances.append(instance)

    # Spacers repeat the same XY hole pattern at each of the three gap heights
    spacer_instances = []
    spacer_groups = [
        (spacer_part_0_1, SPACER_LENGTH_0_to_1, z_spacer_0_1, "spacer_0_1"),
        (spacer_part_1_2, SPACER_LENGTH_1_to_2, z_spacer_1_2, "spacer_1_2"),
        (spacer_part_top, SPACER_LENGTH_TOP, z_spacer_top, "spacer_top"),
    ]
    for part, length, z, name in spacer_groups:
        for i, loc in enumerate(positions_for_spacers):
            instance = (Pos(0, 0, z + length / 2) * loc) * part
            instance.label = f"{name}_{i}"
            spacer_instances.append(instance)

    assembly = Compound(
        label="assembly_body",
        children=[
            body_layer_0_part,
            body_layer_1_part,
            body_layer_2_part,
            *toe_instances,
            *spacer_instances,
        ],
    )

    return assembly


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_body.step")

    show(assembly, name="assembly_body", clear=True)


if __name__ == "__main__":
    main()