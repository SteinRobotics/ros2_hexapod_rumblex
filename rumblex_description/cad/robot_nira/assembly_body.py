#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR

from build123d import Compound, Pos, Rot, export_step
from utils.ocp_utils import show

from utils.colors import COLOR_CREAMY_WHITE, COLOR_WINE_RED, COLOR_DARK_GRAY

import common.battery as battery
import common.toe as toe
import utils.spacer as spacer
import robot_nira.body_common as body_common
import robot_nira.body_layer_0 as body_layer_0
import robot_nira.body_layer_1 as body_layer_1
import robot_nira.body_layer_2 as body_layer_2
import robot_nira.body_layer_2_addon as body_layer_2_addon
import robot_nira.body_layer_2_cover as body_layer_2_cover
import robot_nira.body_layer_3 as body_layer_3
import robot_nira.body_layer_4 as body_layer_4
import robot_nira.chassis_diagonal_back as chassis_diagonal_back
import robot_nira.chassis_side as chassis_side
import robot_nira.chassis_slope_cover as chassis_slope_cover
from robot_nira import board_ydlidar_tmini_interface
from robot_nira import lidar_ydlidar_tmini
from robot_nira import lidar_interface_housing
from robot_nira.lidar_layout import LIDAR_ASSEMBLY_X, LIDAR_Y

from robot_nira import body_layout
from robot_nira.body_layout import (
    SPACER_LENGTH_0_to_1, SPACER_LENGTH_1_to_2, SPACER_LENGTH_TOP,
)

SPACER_OUTER_DIAMETER = 5.0
SPACER_INNER_DIAMETER = 3.0
BATTERY_GAP = 3.0
BATTERY_X = -30.0


def build_assembly() -> Compound:
    body_layer_0_part = body_layer_0.build_model(body_layer_0.build_surface())
    body_layer_1_part = body_layer_1.build_model(body_layer_1.build_surface())
    body_layer_2_part = body_layer_2.build_model(body_layer_2.build_surface())
    body_layer_2_addon_part = body_layer_2_addon.build_model(body_layer_2_addon.build_surface())
    body_layer_2_cover_part = body_layer_2_cover.build_model(body_layer_2_cover.build_surface())
    body_layer_3_part = body_layer_3.build_model(body_layer_3.build_surface())
    body_layer_4_part = body_layer_4.build_model(body_layer_4.build_surface())
    battery_part = battery.build_model()
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
    body_layer_2_addon_part.color = COLOR_CREAMY_WHITE
    body_layer_2_cover_part.color = COLOR_CREAMY_WHITE
    body_layer_3_part.color = COLOR_WINE_RED
    body_layer_4_part.color = COLOR_CREAMY_WHITE
    toe_part.color = COLOR_DARK_GRAY
    spacer_part_0_1.color = COLOR_WINE_RED
    spacer_part_1_2.color = COLOR_WINE_RED
    spacer_part_top.color = COLOR_WINE_RED

    body_layer_0_part.label = "body_layer_0"
    body_layer_1_part.label = "body_layer_1"
    body_layer_2_part.label = "body_layer_2"
    body_layer_2_addon_part.label = "body_layer_2_addon"
    body_layer_2_cover_part.label = "body_layer_2_cover"
    body_layer_3_part.label = "body_layer_3"
    body_layer_4_part.label = "body_layer_4"
    toe_part.label = "toe"

    positions_for_toes = body_layer_0.TOE_MOUNTING_POSITIONS
    positions_for_spacers = body_common.hole_locations

    # --- Z-stack the layers, separated by spacer lengths ---
    z_layer_1 = body_layout.LAYER_1_BOTTOM
    z_layer_2 = body_layout.LAYER_2_BOTTOM
    z_layer_3 = body_layout.LAYER_3_BOTTOM
    z_spacer_0_1 = body_layout.LAYER_0_TOP
    z_spacer_1_2 = body_layout.LAYER_1_TOP
    z_spacer_top = body_layout.LAYER_2_TOP

    # Face the board connector and housing cable opening toward the robot center (-X).
    lidar_interface_flip = Rot(Z=180)
    board_interface = (
        Pos(LIDAR_ASSEMBLY_X, LIDAR_Y, z_spacer_top)
        * lidar_interface_flip
        * Rot(Z=90)
        * board_ydlidar_tmini_interface.build_model()
    )
    board_interface.label = "board_ydlidar_tmini_interface"

    housing = (
        Pos(LIDAR_ASSEMBLY_X, LIDAR_Y, z_spacer_top)
        * lidar_interface_flip
        * lidar_interface_housing.build_model()
    )

    lidar = Pos(
        LIDAR_ASSEMBLY_X,
        LIDAR_Y,
        z_spacer_top + lidar_interface_housing.HEIGHT,
    ) * lidar_ydlidar_tmini.build_model()
    lidar.label = "lidar_ydlidar_tmini"

    body_layer_1_part = Pos(0, 0, z_layer_1) * body_layer_1_part
    body_layer_1_part.label = "body_layer_1"
    body_layer_2_part = Pos(0, 0, z_layer_2) * body_layer_2_part
    body_layer_2_part.label = "body_layer_2"
    body_layer_2_addon_part = Pos(0, 0, z_layer_2 - body_layer_2_addon.THICKNESS) * body_layer_2_addon_part
    body_layer_2_addon_part.label = "body_layer_2_addon"
    z_cover_top = body_layout.COVER_BOTTOM + body_layer_2_cover.THICKNESS
    body_layer_2_cover_part = Pos(0, 0, body_layout.COVER_BOTTOM) * body_layer_2_cover_part
    body_layer_2_cover_part.label = "body_layer_2_cover"
    battery_y = (battery.BODY_WIDTH + BATTERY_GAP) / 2
    battery_left = Pos(BATTERY_X, battery_y, z_cover_top) * battery_part
    battery_left.label = "battery_left"
    battery_right = Pos(BATTERY_X, -battery_y, z_cover_top) * battery_part
    battery_right.label = "battery_right"
    body_layer_3_part = Pos(0, 0, z_layer_3) * body_layer_3_part
    body_layer_3_part.label = "body_layer_3"
    body_layer_4_part = Pos(0, 0, body_layout.LAYER_4_BOTTOM) * body_layer_4_part
    body_layer_4_part.label = "body_layer_4"

    # Seat the toe mounting faces against the underside of layer 0.
    toe_instances = []
    inverted_toe = Rot(X=180) * toe_part
    for i, (x, y) in enumerate(positions_for_toes):
        instance = Pos(x, y, body_layout.LAYER_0_BOTTOM - inverted_toe.bounding_box().max.Z) * inverted_toe
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
            if name == "spacer_top" and loc.position.X > 0:
                continue
            instance = (Pos(0, 0, z + length / 2) * loc) * part
            instance.label = f"{name}_{i}"
            spacer_instances.append(instance)

    assembly = Compound(
        label="assembly_body",
        children=[
            body_layer_0_part,
            body_layer_1_part,
            body_layer_2_part,
            body_layer_2_addon_part,
            body_layer_2_cover_part,
            battery_left,
            battery_right,
            body_layer_3_part,
            body_layer_4_part,
            board_interface,
            housing,
            lidar,
            *chassis_side.build_plates(z_layer_1, z_layer_3),
            *chassis_diagonal_back.build_diagonal_plates(z_layer_2, z_layer_3),
            chassis_slope_cover.build_slope_cover(z_layer_3),
            *chassis_slope_cover.build_speakers(z_layer_3),
            *toe_instances,
            *spacer_instances,
        ],
    )

    return assembly


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / "step/assembly_body.step"))

    show(assembly, name="assembly_body", clear=True)


if __name__ == "__main__":
    main()
