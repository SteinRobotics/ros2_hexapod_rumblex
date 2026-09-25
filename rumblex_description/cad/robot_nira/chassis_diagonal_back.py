#!/usr/bin/env python3
"""Nira diagonal back chassis plate."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import ExportDXF, Part, Pos, Rot, Sketch, export_step
from robot_nira import EXPORT_DIR
from robot_nira import chassis_common
import robot_nira.body_common as body_common
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show


def build_surface(height: float) -> Sketch:
    return chassis_common.build_plate_surface(height)


def build_model(height: float) -> Part:
    return chassis_common.build_model(build_surface(height))


def build_diagonal_plates(z_bottom: float, z_top: float) -> list[Part]:
    """Place rear and front diagonal braces in their slots."""
    from robot_nira import chassis_diagonal_front

    height = z_top + body_common.THICKNESS - z_bottom
    rear = build_model(height)
    front = chassis_diagonal_front.build_model(height)
    upright = Pos(0, chassis_common.THICKNESS / 2, 0) * Rot(X=90)
    plates = []
    locations = [*(location for location in body_common.diagonal_slots_locations
                   if location.position.X < 0),
                 *body_common.front_diagonal_slots_locations]
    for location in locations:
        is_front = location.position.X > 0
        plate = Pos(0, 0, z_bottom) * location * upright * (front if is_front else rear)
        end = "front" if is_front else "back"
        side = "left" if location.position.Y > 0 else "right"
        plate.label = f"chassis_diagonal_{end}_{side}"
        plate.color = COLOR_CREAMY_WHITE
        plates.append(plate)
    return plates


def main() -> None:
    height = body_common.SPACER_LENGTH_TOP + 2 * chassis_common.TAB_DEPTH
    surface = build_surface(height)
    model = chassis_common.build_model(surface)
    model.label = "chassis_diagonal_back"
    step_dir = EXPORT_DIR / "step"
    dxf_dir = EXPORT_DIR / "dxf"
    step_dir.mkdir(parents=True, exist_ok=True)
    dxf_dir.mkdir(parents=True, exist_ok=True)
    export_step(model, str(step_dir / "chassis_diagonal_back.step"))
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.write(str(dxf_dir / "chassis_diagonal_back.dxf"))
    show(model, name="chassis_diagonal_back", clear=True)


if __name__ == "__main__":
    main()
