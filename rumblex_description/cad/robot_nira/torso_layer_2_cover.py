#!/usr/bin/env python3
"""Cover for the central opening in Nira's second body layer."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    BuildPart,
    BuildSketch,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Pos,
    Rectangle,
    Sketch,
    add,
    export_step,
    extrude,
    offset,
)

from robot_nira import EXPORT_DIR, dxf_directory
import robot_nira.torso_common as torso_common
from robot_nira.torso_layout import FRONT_DECK_X as FRONT_EDGE_X
from utils.ocp_utils import show


THICKNESS = 1.5
EDGE_CLEARANCE = 0.2
SIDE_CABLE_NOTCH_WIDTH = 14.0
END_CABLE_NOTCH_WIDTH = 54.0  # Both former 10 mm slots at y=+/-20, plus 4 mm
CABLE_NOTCH_DEPTH = 10.0


def build_surface() -> Sketch:
    """Follow the opening with one end notch and two side notches per edge."""
    outline = build_opening_outline()
    cable_cutouts = build_cable_cutouts(outline)
    with BuildSketch() as sketch:
        add(outline)
        add(cable_cutouts, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_opening_outline() -> Sketch:
    bounds = torso_common.hantel.sketch.bounding_box()
    rear_of_deck = Pos((bounds.min.X + FRONT_EDGE_X) / 2, bounds.center().Y) * Rectangle(
        FRONT_EDGE_X - bounds.min.X, bounds.size.Y)
    opening = torso_common.hantel.sketch & rear_of_deck
    return offset(opening, amount=-EDGE_CLEARANCE)


def build_cable_cutouts(outline: Sketch, *, stop_at_edge: bool = False) -> Sketch:
    """Cable cutters; keep the addon's outer rim when stopping at the edge."""
    front_x = outline.bounding_box().max.X
    rear_x = outline.bounding_box().min.X
    side_y = outline.bounding_box().max.Y
    depth = CABLE_NOTCH_DEPTH

    with BuildSketch() as sketch:
        if stop_at_edge:
            with Locations((front_x - depth / 2, 0), (rear_x + depth / 2, 0)):
                Rectangle(depth, END_CABLE_NOTCH_WIDTH)
            with Locations(*[(x, sign * (side_y - depth / 2))
                             for x in (-45.0, 15.0) for sign in (-1, 1)]):
                Rectangle(SIDE_CABLE_NOTCH_WIDTH, depth)
        else:
            # Extend beyond the cover edges so the openings are notches.
            with Locations((front_x, 0), (rear_x, 0)):
                Rectangle(2 * depth, END_CABLE_NOTCH_WIDTH)
            with Locations(*[(x, y) for x in (-45.0, 15.0) for y in (-side_y, side_y)]):
                Rectangle(SIDE_CABLE_NOTCH_WIDTH, 2 * depth)

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
    dxf_dir = dxf_directory(THICKNESS)
    export_step(result, str(EXPORT_DIR / "step/torso_layer_2_cover.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(dxf_dir / "torso_layer_2_cover.dxf"))

    show(result, name="torso_layer_2_cover", clear=True)


if __name__ == "__main__":
    main()
