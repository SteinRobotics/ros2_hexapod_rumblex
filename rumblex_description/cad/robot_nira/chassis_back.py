#!/usr/bin/env python3
"""Nira back chassis plate."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import ExportDXF, Part, Sketch, export_step
from robot_nira import EXPORT_DIR
from robot_nira import chassis_common
import robot_nira.torso_common as torso_common
from utils.ocp_utils import show


def build_surface(height: float) -> Sketch:
    return chassis_common.build_plate_surface(height)


def build_model(height: float) -> Part:
    return chassis_common.build_model(build_surface(height))


def main() -> None:
    from robot_nira.torso_layout import SIDE_PLATE_HEIGHT

    height = SIDE_PLATE_HEIGHT
    surface = build_surface(height)
    model = chassis_common.build_model(surface)
    model.label = "chassis_back"
    step_dir = EXPORT_DIR / "step"
    dxf_dir = EXPORT_DIR / "dxf"
    step_dir.mkdir(parents=True, exist_ok=True)
    dxf_dir.mkdir(parents=True, exist_ok=True)
    export_step(model, str(step_dir / "chassis_back.step"))
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.write(str(dxf_dir / "chassis_back.dxf"))
    show(model, name="chassis_back", clear=True)


if __name__ == "__main__":
    main()
