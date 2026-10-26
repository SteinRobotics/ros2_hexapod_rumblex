#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR, dxf_directory
from robot_nira.torso_layout import FRONT_DECK_X
from build123d import *
from build123d import ColorIndex
from utils.ocp_utils import show

import robot_nira.torso_common as torso_common
import robot_nira.torso_layer_2_addon as torso_layer_2_addon
from robot_nira.lidar_layout import LIDAR_X, LIDAR_ASSEMBLY_X, LIDAR_Y
from robot_nira.lidar_interface_housing import (
    DECK_TAB_POSITIONS, DECK_TAB_WIDTH, WALL, SPACER_POSITIONS, SPACER_INNER_DIAMETER,
)

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        insert(torso_common.build_surface())


        # Integral front deck bridges the old central opening to both side rails.
        Polygon((FRONT_DECK_X, -50), (78, -50), (109, -25), (109, 25),
                (78, 50), (FRONT_DECK_X, 50), align=None)

        # Three fingers on each housing side wall pass through matching deck slots.
        with Locations(*[(LIDAR_ASSEMBLY_X + x, y) for x, y in DECK_TAB_POSITIONS]):
            Rectangle(DECK_TAB_WIDTH, WALL, mode=Mode.SUBTRACT)

        # The housing is rotated 180 degrees on this deck.
        with Locations(*[(LIDAR_ASSEMBLY_X - x, LIDAR_Y - y)
                         for x, y in SPACER_POSITIONS]):
            Circle(SPACER_INNER_DIAMETER / 2, mode=Mode.SUBTRACT)

        # Only the rear diagonal braces need slots in the deck.
        with Locations(*(location for location in torso_common.diagonal_slots_locations
                         if location.position.X < 0)):
            insert(torso_common.rectangle_slots.sketch, mode=Mode.SUBTRACT)

        # The front sill ends at this deck, so its three top fingers need
        # individual slots. The other rows retain their continuous openings.
        front_location = max(torso_common.rectangle_slots_locations,
                             key=lambda location: location.position.X)
        with Locations(front_location):
            insert(torso_common.rectangle_slots.sketch, mode=Mode.SUBTRACT)
        with Locations(*(location for location in torso_common.rectangle_slots_locations
                         if location != front_location)):
            Rectangle(
                torso_common.rectangle_slots_completed_width + torso_common.rectangle_slots_width,
                torso_common.rectangle_slots_height,
                mode=Mode.SUBTRACT,
            )

        for points in torso_common.SERVO_FRONT_CUTOUTS.values():
            Polygon(*points, align=None, mode=Mode.SUBTRACT)

        for x, y, radius in torso_common.LIST_SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        insert(surface)
        extrude(amount=torso_common.THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    dxf_dir = dxf_directory(torso_common.THICKNESS)
    export_step(result, str(EXPORT_DIR / "step/torso_layer_2.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    # Engrave on the underside to locate the addon during assembly.
    addon_surface = torso_layer_2_addon.build_surface()
    dxf_export.add_layer("ENGRAVE", color=ColorIndex.BLUE)
    for face in addon_surface.faces():
        dxf_export.add_shape(face.outer_wire(), layer="ENGRAVE")
    dxf_export.write(str(dxf_dir / "torso_layer_2.dxf"))

    show(result, name="torso_layer_2", clear=True)

        
if __name__ == "__main__":
    main()
