#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR
from build123d import *
from utils.ocp_utils import show

import robot_nira.body_common as body_common
from robot_nira.lidar_layout import LIDAR_X, LIDAR_ASSEMBLY_X
from robot_nira.lidar_interface_housing import DECK_TAB_POSITIONS, DECK_TAB_WIDTH, WALL

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(body_common.build_surface())


        # Integral front deck bridges the old central opening to both side rails.
        Polygon((55, -50), (78, -50), (102, -25), (102, 25),
                (78, 50), (55, 50), align=None)

        # Three fingers on each housing side wall pass through matching deck slots.
        with Locations(*[(LIDAR_ASSEMBLY_X + x, y) for x, y in DECK_TAB_POSITIONS]):
            Rectangle(DECK_TAB_WIDTH, WALL, mode=Mode.SUBTRACT)

        # Rear braces keep three tabs; the front pair use shorter joints
        # farther outboard to clear the interface housing.
        with Locations(*(location for location in body_common.diagonal_slots_locations
                         if location.position.X < 0)):
            add(body_common.rectangle_slots.sketch, mode=Mode.SUBTRACT)
        with Locations(*body_common.front_diagonal_slots_locations):
            add(body_common.front_diagonal_slots.sketch, mode=Mode.SUBTRACT)

        # The front sill ends at this deck, so its three top fingers need
        # individual slots. The other rows retain their continuous openings.
        front_location = max(body_common.rectangle_slots_locations,
                             key=lambda location: location.position.X)
        with Locations(front_location):
            add(body_common.rectangle_slots.sketch, mode=Mode.SUBTRACT)
        with Locations(*(location for location in body_common.rectangle_slots_locations
                         if location != front_location)):
            Rectangle(
                body_common.rectangle_slots_completed_width + body_common.rectangle_slots_width,
                body_common.rectangle_slots_height,
                mode=Mode.SUBTRACT,
            )

        for points in body_common.SERVO_FRONT_CUTOUTS.values():
            Polygon(*points, align=None, mode=Mode.SUBTRACT)

        for x, y, radius in body_common.LIST_SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=body_common.THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "dxf").mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "step/body_layer_2.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(EXPORT_DIR / "dxf/body_layer_2.dxf"))

    show(result, name="body_layer_2", clear=True)

        
if __name__ == "__main__":
    main()
