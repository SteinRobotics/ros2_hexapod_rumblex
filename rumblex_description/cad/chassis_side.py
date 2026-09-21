#!/usr/bin/env python3
"""Flat rectangular end plates and inverted-U side plates, with body-slot tabs.

Sketch X runs along the slot rows; sketch Y is the assembled height. The
bottom tab tips are at Y=0. All fits are nominal, without kerf compensation.
"""

from pathlib import Path

from build123d import (
    BuildPart, BuildSketch, Compound, ExportDXF, Locations, Part, Pos,
    Rectangle, Rot, Sketch, add, export_step, extrude,
)

import body_common
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show


THICKNESS = body_common.rectangle_slots_height
TAB_WIDTH = body_common.rectangle_slots_width
TAB_DEPTH = body_common.THICKNESS
TAB_PITCH = body_common.rectangle_slots_completed_width / 2
ROW_WIDTH = 2 * TAB_PITCH + TAB_WIDTH
# The rail sits on layer 2 and reaches the underside of layer 3.
RAIL_HEIGHT = body_common.SPACER_LENGTH_TOP


def build_surface(height: float, side: bool = False) -> Sketch:
    """Build a profile spanning the outside faces of the two mating layers.

    ``side=False`` selects the front/back rectangle. ``side=True`` selects
    the inverted U, joining the two side slot rows across its top.
    """
    if height <= 2 * TAB_DEPTH + (RAIL_HEIGHT if side else 0):
        raise ValueError("Height must leave room for the plate below the top rail")

    side_rows = sorted(
        loc.position.X for loc in body_common.rectangle_slots_locations
        if loc.position.Y > 0 and abs(loc.orientation.Z) < 1e-6
    )
    row_centers = side_rows if side else [0.0]
    with BuildSketch() as sketch:
        if side:
            # Full-width legs pass through the elongated slots in layer 2.
            with Locations(*[(x, height / 2) for x in row_centers]):
                Rectangle(ROW_WIDTH, height - 2 * TAB_DEPTH)
            with Locations(((side_rows[0] + side_rows[-1]) / 2,
                            height - TAB_DEPTH - RAIL_HEIGHT / 2)):
                Rectangle(side_rows[-1] - side_rows[0] + ROW_WIDTH, RAIL_HEIGHT)
        else:
            with Locations((0, height / 2)):
                Rectangle(ROW_WIDTH, height - 2 * TAB_DEPTH)

        for row in row_centers:
            for offset in (-TAB_PITCH, 0, TAB_PITCH):
                with Locations((row + offset, TAB_DEPTH / 2),
                               (row + offset, height - TAB_DEPTH / 2)):
                    Rectangle(TAB_WIDTH, TAB_DEPTH)
    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)
    return model.part


def build_plates(z_bottom: float, z_top: float) -> list[Part]:
    """Place four plates in the shared slots of the two given layer bases."""
    height = z_top + body_common.THICKNESS - z_bottom
    end = build_model(build_surface(height))
    side = build_model(build_surface(height, side=True))
    plates = []
    # Local extrusion points towards -Y after the upright rotation. Shift
    # half a thickness so each plate is centred on its slot row.
    upright = Pos(0, THICKNESS / 2, 0) * Rot(X=90)
    for loc in body_common.rectangle_slots_locations:
        if abs(loc.orientation.Z) > 1e-6:
            name = "front" if loc.position.X > 0 else "back"
            plate = Pos(0, 0, z_bottom) * loc * upright * end
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
    # Import here so assembly_body can use the builders without a cycle.
    import assembly_body

    height = (3 * body_common.THICKNESS + assembly_body.SPACER_LENGTH_1_to_2
              + assembly_body.SPACER_LENGTH_TOP)
    output = Path("generated")
    output.mkdir(exist_ok=True)
    previews = []
    for name, is_side in (("chassis_side_end", False), ("chassis_side", True)):
        surface = build_surface(height, side=is_side)
        model = build_model(surface)
        model.label = name
        export_step(model, str(output / f"{name}.step"))
        drawing = ExportDXF()
        drawing.add_shape(surface)
        drawing.write(str(output / f"{name}.dxf"))
        previews.append(Pos(160 if is_side else 0, 0, 0) * model)
    show(Compound(children=previews), name="chassis_side", clear=True)


if __name__ == "__main__":
    main()
