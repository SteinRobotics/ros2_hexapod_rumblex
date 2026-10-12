#!/usr/bin/env python3
"""Nira back chassis plate."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import Circle, ExportDXF, Part, Pos, Sketch
from robot_nira import EXPORT_DIR, dxf_directory
from robot_nira import chassis_common
import robot_nira.torso_common as torso_common
from utils.ocp_utils import show

from robot_nira.torso_layout import (
     SPACER_LENGTH_1_to_2
)


SWITCH_RADIUS = 10.0


def build_surface(height: float) -> Sketch:
    """Build the rear plate with a switch hole midway between layers 1 and 2."""
    surface = chassis_common.build_plate_surface(height)
    center_y = chassis_common.TAB_DEPTH + SPACER_LENGTH_1_to_2 / 2
    return surface - Pos(0, center_y) * Circle(SWITCH_RADIUS)


def build_model(height: float) -> Part:
    return chassis_common.build_model(build_surface(height))


def main() -> None:
    from robot_nira.torso_layout import SIDE_PLATE_HEIGHT

    height = SIDE_PLATE_HEIGHT
    surface = build_surface(height)
    model = chassis_common.build_model(surface)
    model.label = "chassis_back"
    dxf_dir = dxf_directory(chassis_common.THICKNESS)
    dxf_dir.mkdir(parents=True, exist_ok=True)
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.write(str(dxf_dir / "chassis_back.dxf"))
    show(model, name="chassis_back", clear=True)


if __name__ == "__main__":
    main()
