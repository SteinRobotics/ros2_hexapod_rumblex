#!/usr/bin/env python3
"""Nira diagonal front chassis plate."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (BuildSketch, ExportDXF, Locations, Part, Polygon,
                       Rectangle, Sketch, export_step)
from robot_nira import EXPORT_DIR
from robot_nira import chassis_common
import robot_nira.body_common as body_common
from utils.ocp_utils import show

FRONT_DIAGONAL_HEIGHT = 10.0


def build_surface(height: float) -> Sketch:
    """Compact chamfered brace with two tabs for the front deck."""
    if height <= FRONT_DIAGONAL_HEIGHT:
        raise ValueError("Height must leave room for the front diagonal brace")
    tab_depth = chassis_common.TAB_DEPTH
    with BuildSketch() as sketch:
        Polygon(
            (0, tab_depth), (0, tab_depth),
            (13, tab_depth), (13, FRONT_DIAGONAL_HEIGHT - 3),
            (10, FRONT_DIAGONAL_HEIGHT), (-10, FRONT_DIAGONAL_HEIGHT),
            (-13, FRONT_DIAGONAL_HEIGHT - 3), (-13, tab_depth),
            align=None,
        )
        with Locations(*[(x, tab_depth / 2)
                         for x in body_common.front_diagonal_tab_offsets]):
            Rectangle(body_common.front_diagonal_tab_width, tab_depth)
    return sketch.sketch


def build_model(height: float) -> Part:
    return chassis_common.build_model(build_surface(height))


def main() -> None:
    import robot_nira.assembly_body as assembly_body

    height = body_common.SPACER_LENGTH_TOP + 2 * chassis_common.TAB_DEPTH
    surface = build_surface(height)
    model = chassis_common.build_model(surface)
    model.label = "chassis_diagonal_front"
    step_dir = EXPORT_DIR / "step"
    dxf_dir = EXPORT_DIR / "dxf"
    step_dir.mkdir(parents=True, exist_ok=True)
    dxf_dir.mkdir(parents=True, exist_ok=True)
    export_step(model, str(step_dir / "chassis_diagonal_front.step"))
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.write(str(dxf_dir / "chassis_diagonal_front.dxf"))
    show(model, name="chassis_diagonal_front", clear=True)


if __name__ == "__main__":
    main()
