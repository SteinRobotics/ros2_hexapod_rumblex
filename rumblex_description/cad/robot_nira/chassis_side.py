#!/usr/bin/env python3
"""Nira side chassis plate."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (BuildSketch, ExportDXF, Locations, Mode, Part, Polygon,
                       Pos, Rectangle, Rot, Sketch, export_step)
from robot_nira import EXPORT_DIR
from robot_nira import chassis_common
import robot_nira.body_common as body_common
from robot_nira.armor_style import gill_points
from robot_nira.lidar_layout import LOW_RAIL_HEIGHT, SHOULDER_TOP_X
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show


def build_surface(height: float) -> Sketch:
    """Swept side shoulder with two tabbed legs and rear gills."""
    tab_depth = chassis_common.TAB_DEPTH
    rail_height = chassis_common.RAIL_HEIGHT
    if height <= 2 * tab_depth + rail_height:
        raise ValueError("Height must leave room for the plate below the top rail")

    side_rows = sorted(
        loc.position.X for loc in body_common.rectangle_slots_locations
        if loc.position.Y > 0 and abs(loc.orientation.Z) < 1e-6
    )
    with BuildSketch() as sketch:
        # Full-width legs pass through the elongated slots in layer 2.
        with Locations(*[(x, height / 2) for x in side_rows]):
            Rectangle(chassis_common.ROW_WIDTH, height - 2 * tab_depth)
        with Locations(((side_rows[0] + side_rows[-1]) / 2,
                        height - tab_depth - rail_height / 2)):
            Rectangle(side_rows[-1] - side_rows[0] + chassis_common.ROW_WIDTH,
                      rail_height)
        for row in side_rows:
            for offset in (-chassis_common.TAB_PITCH, 0, chassis_common.TAB_PITCH):
                with Locations((row + offset, tab_depth / 2),
                               (row + offset, height - tab_depth / 2)):
                    Rectangle(chassis_common.TAB_WIDTH, tab_depth)

        sill = height - tab_depth - rail_height + LOW_RAIL_HEIGHT
        Polygon((SHOULDER_TOP_X, height - tab_depth),
                (chassis_common.SLOPE_END_X, sill), (150, sill),
                (150, height + 1), (SHOULDER_TOP_X, height + 1),
                align=None, mode=Mode.SUBTRACT)
        for profile in chassis_common.slope_finger_profiles(height - tab_depth):
            Polygon(*profile, align=None, mode=Mode.SUBTRACT)
        gill_span = 7.0
        gill_y = (sill + 5 + height - 9 - gill_span) / 2
        for x in (-29, -17, -5):
            Polygon(*gill_points(x, gill_y, gill_span),
                    align=None, mode=Mode.SUBTRACT)
    return sketch.sketch


def build_model(height: float) -> Part:
    return chassis_common.build_model(build_surface(height))


def build_plates(z_bottom: float, z_top: float) -> list[Part]:
    """Place front, back, and side plates in the shared slots."""
    from robot_nira import chassis_back, chassis_front

    height = z_top + body_common.THICKNESS - z_bottom
    end = chassis_back.build_model(height)
    nose = chassis_front.build_model(height)
    side = build_model(height)
    plates = []
    upright = Pos(0, chassis_common.THICKNESS / 2, 0) * Rot(X=90)
    for loc in body_common.rectangle_slots_locations:
        if abs(loc.orientation.Z) > 1e-6:
            name = "front" if loc.position.X > 0 else "back"
            plate = Pos(0, 0, z_bottom) * loc * upright * (nose if name == "front" else end)
        elif loc.position.X > 0:
            name = "left" if loc.position.Y > 0 else "right"
            plate = Pos(0, loc.position.Y, z_bottom) * upright * side
        else:
            continue
        plate.label = f"chassis_{name}"
        plate.color = COLOR_CREAMY_WHITE
        plates.append(plate)
    return plates


def main() -> None:
    import robot_nira.assembly_body as assembly_body

    height = (3 * body_common.THICKNESS + assembly_body.SPACER_LENGTH_1_to_2
              + assembly_body.SPACER_LENGTH_TOP)
    surface = build_surface(height)
    model = chassis_common.build_model(surface)
    model.label = "chassis_side"
    step_dir = EXPORT_DIR / "step"
    dxf_dir = EXPORT_DIR / "dxf"
    step_dir.mkdir(parents=True, exist_ok=True)
    dxf_dir.mkdir(parents=True, exist_ok=True)
    export_step(model, str(step_dir / "chassis_side.step"))
    drawing = ExportDXF()
    drawing.add_shape(surface)
    drawing.write(str(dxf_dir / "chassis_side.dxf"))
    show(model, name="chassis_side", clear=True)


if __name__ == "__main__":
    main()
