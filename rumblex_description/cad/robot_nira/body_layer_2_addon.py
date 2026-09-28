#!/usr/bin/env python3
"""Thin U-shaped plate mounted against the underside of body layer 2."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import BuildPart, BuildSketch, ExportDXF, Part, Polygon, Sketch, add, export_step, extrude

from robot_nira import EXPORT_DIR
import robot_nira.body_common as body_common
from utils.ocp_utils import show


THICKNESS = 1.0
FINGER_WIDTH = 6.0
WIDTH = body_common.inner_rectangle_width + 12
REAR_X = -68.0
BASE_INNER_X = 52.0
FRONT_X = BASE_INNER_X + FINGER_WIDTH


def build_surface() -> Sketch:
    """Make the open rear profile; +X points toward the U's closed end."""
    half_width = WIDTH / 2
    inner_y = half_width - FINGER_WIDTH
    with BuildSketch() as sketch:
        Polygon(
            (REAR_X, -half_width),
            (FRONT_X, -half_width),
            (FRONT_X, half_width),
            (REAR_X, half_width),
            (REAR_X, inner_y),
            (BASE_INNER_X, inner_y),
            (BASE_INNER_X, -inner_y),
            (REAR_X, -inner_y),
            align=None,
        )
    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)
    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "dxf").mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "step/body_layer_2_addon.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(EXPORT_DIR / "dxf/body_layer_2_addon.dxf"))

    show(result, name="body_layer_2_addon", clear=True)


if __name__ == "__main__":
    main()
