#!/usr/bin/env python3
"""Nira front chassis plate."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (BuildSketch, ExportDXF, Locations, Mode, Part, Rectangle,
                       Sketch, add, export_step)
from robot_nira import EXPORT_DIR
from robot_nira import chassis_common
import robot_nira.body_common as body_common
from utils.ocp_utils import show


def build_surface(height: float) -> Sketch:
    """Low front sill with three fingers that meet the middle deck."""
    base = chassis_common.build_plate_surface(height)
    deck_top = height - chassis_common.TAB_DEPTH - chassis_common.RAIL_HEIGHT
    top_of_body = deck_top - chassis_common.TAB_DEPTH
    with BuildSketch() as sketch:
        add(base)
        with Locations((0, (top_of_body + height + 2) / 2)):
            Rectangle(chassis_common.ROW_WIDTH + 2, height + 2 - top_of_body,
                      mode=Mode.SUBTRACT)
        with Locations(*[(offset, deck_top - chassis_common.TAB_DEPTH / 2)
                         for offset in (-chassis_common.TAB_PITCH, 0,
                                        chassis_common.TAB_PITCH)]):
            Rectangle(chassis_common.TAB_WIDTH, chassis_common.TAB_DEPTH)
    return sketch.sketch


def build_model(height: float) -> Part:
    return chassis_common.build_model(build_surface(height))


def main() -> None:
    import robot_nira.assembly_body as assembly_body

    height = (3 * body_common.THICKNESS + assembly_body.SPACER_LENGTH_1_to_2
              + assembly_body.SPACER_LENGTH_TOP)
    surface = build_surface(height)
    model = chassis_common.build_model(surface)
    model.label = "chassis_front"
    step_dir = EXPORT_DIR / "step"
    dxf_dir = EXPORT_DIR / "dxf"
    step_dir.mkdir(parents=True, exist_ok=True)
    dxf_dir.mkdir(parents=True, exist_ok=True)
    export_step(model, str(step_dir / "chassis_front.step"))
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.write(str(dxf_dir / "chassis_front.dxf"))
    show(model, name="chassis_front", clear=True)


if __name__ == "__main__":
    main()
