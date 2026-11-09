#!/usr/bin/env python3
"""3 mm inner backing plate for the rear switch opening."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import Circle, ExportDXF, Part, Pos, Rectangle, Rot, Sketch, extrude, export_step
from robot_nira import EXPORT_DIR, chassis_back, chassis_common, torso_common, torso_layout, torso_layer_2_addon, dxf_directory
from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show

THICKNESS = 3.0
WIDTH = chassis_common.ROW_WIDTH
# Leave clearance above the plate for the layer 2 support ring.
EDGE_CLEARANCE = torso_layer_2_addon.THICKNESS
HEIGHT = torso_layout.SPACER_LENGTH_1_to_2 - EDGE_CLEARANCE
HOLE_DIAMETER = 2 * chassis_back.SWITCH_RADIUS


def build_surface() -> Sketch:
    """Keep the opening at the rear hole center and extend down to layer 1."""
    return Pos(0, -EDGE_CLEARANCE / 2) * Rectangle(WIDTH, HEIGHT) - Circle(HOLE_DIAMETER / 2)


def build_model() -> Part:
    return extrude(build_surface(), amount=THICKNESS)


def build_plate() -> Part:
    """Seat on layer 1, flush on the inner (+X) face of the rear panel."""
    rear_slot = next(
        loc for loc in torso_common.rectangle_slots_locations
        if loc.position.X < 0 and abs(loc.orientation.Z) > 1e-6
    )
    # Match chassis_side's upright frame: local +Z becomes global +X.
    plate = (
        Pos(0, 0, torso_layout.LAYER_1_TOP + torso_layout.SPACER_LENGTH_1_to_2 / 2)
        * rear_slot
        * Pos(0, -chassis_common.THICKNESS / 2, 0)
        * Rot(X=90)
        * build_model()
    )
    plate.label = "chassis_back_inner"
    plate.color = COLOR_DARK_GRAY
    return plate


def main() -> None:
    model = build_model()
    model.label = "chassis_back_inner"
    step_dir = EXPORT_DIR / "step"
    step_dir.mkdir(parents=True, exist_ok=True)
    export_step(model, str(step_dir / "chassis_back_inner.step"))
    drawing = ExportDXF()
    drawing.add_shape(build_surface())
    drawing.write(str(dxf_directory(THICKNESS) / "chassis_back_inner.dxf"))
    show(model, name=model.label, clear=True)


if __name__ == "__main__":
    main()
