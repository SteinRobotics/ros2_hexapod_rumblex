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
from robot_nira.lidar_interface_housing import DECK_TAB_X, DECK_TAB_WIDTH, DECK_SLOT_Y, WALL

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(body_common.build_surface())

        # Integral front deck bridges the old central opening to both side rails.
        Polygon((22, -49), (78, -49), (102, -25), (102, 25),
                (78, 49), (22, 49), align=None)
        # Generic adapter/retention slots, outside the simplified sensor envelope.
        # These are not the manufacturer's mounting-hole pattern.
        with Locations((LIDAR_X, -23), (LIDAR_X, 23)):
            RectangleRounded(22, 3, 1, mode=Mode.SUBTRACT)
        with Locations((30, 0)):
            RectangleRounded(6, 16, 2, mode=Mode.SUBTRACT)
        # Slots receive the housing side-wall fingers through this deck.
        with Locations(*[(LIDAR_ASSEMBLY_X + x, y)
                         for x in DECK_TAB_X for y in (-DECK_SLOT_Y, DECK_SLOT_Y)]):
            Rectangle(DECK_TAB_WIDTH, WALL, mode=Mode.SUBTRACT)

        # These plates start at layer 2, so retain three individual tab slots.
        with Locations(*body_common.diagonal_slots_locations):
            add(body_common.rectangle_slots.sketch, mode=Mode.SUBTRACT)

        # Join each three-slot row while retaining its full outer span.
        with Locations(*body_common.rectangle_slots_locations):
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
