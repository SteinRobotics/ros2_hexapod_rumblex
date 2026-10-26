#!/usr/bin/env python3
"""Nira diagonal back chassis plate."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import ExportDXF, Part, Pos, Rectangle, Rot, Sketch
from robot_nira import EXPORT_DIR, dxf_directory
from robot_nira import chassis_common
import robot_nira.torso_common as torso_common
from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show


OLED_WIDTH = 23.0
OLED_HEIGHT = 14.0


def build_surface(height: float, *, with_oled: bool = False) -> Sketch:
    surface = chassis_common.build_plate_surface(height)
    if with_oled:
        face_height = height/2 + 10
        center_y = chassis_common.TAB_DEPTH + 5 * face_height / 6
        surface -= Pos(0, center_y) * Rectangle(OLED_WIDTH, OLED_HEIGHT)
    return surface


def build_model(height: float, *, with_oled: bool = False) -> Part:
    return chassis_common.build_model(build_surface(height, with_oled=with_oled))


def build_diagonal_plates(z_bottom: float, z_top: float) -> list[Part]:
    """Place the rear diagonal braces in their slots."""
    height = z_top + torso_common.THICKNESS - z_bottom
    rear = build_model(height)
    rear_left = build_model(height, with_oled=True)
    upright = Pos(0, chassis_common.THICKNESS / 2, 0) * Rot(X=90)
    plates = []
    locations = [location for location in torso_common.diagonal_slots_locations
                 if location.position.X < 0]
    for location in locations:
        is_left = location.position.Y > 0
        model = rear_left if is_left else rear
        plate = Pos(0, 0, z_bottom) * location * upright * model
        side = "left" if location.position.Y > 0 else "right"
        plate.label = f"chassis_diagonal_back_{side}"
        plate.color = COLOR_DARK_GRAY
        plates.append(plate)
    return plates


def main() -> None:
    from robot_nira.torso_layout import DIAGONAL_PLATE_HEIGHT

    height = DIAGONAL_PLATE_HEIGHT
    dxf_dir = dxf_directory(chassis_common.THICKNESS)
    dxf_dir.mkdir(parents=True, exist_ok=True)
    for name, with_oled in (("chassis_diagonal_back", False),
                            ("chassis_diagonal_back_left", True)):
        surface = build_surface(height, with_oled=with_oled)
        drawing = ExportDXF()
        drawing.add_shape(surface)
        drawing.write(str(dxf_dir / f"{name}.dxf"))
    model = build_model(height, with_oled=True)
    model.label = "chassis_diagonal_back_left"
    show(model, name=model.label, clear=True)


if __name__ == "__main__":
    main()
