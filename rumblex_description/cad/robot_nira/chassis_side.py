#!/usr/bin/env python3
"""Low front sills and swept, vented rear shoulders with body-slot tabs.

Sketch X runs along the slot rows; sketch Y is the assembled height. The
bottom tab tips are at Y=0. All fits are nominal, without kerf compensation.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR

from build123d import (
    BuildPart, BuildSketch, Compound, ExportDXF, Locations, Part, Pos,
    Rectangle, Rot, Sketch, Mode, Polygon, add, export_step, extrude,
)

import robot_nira.body_common as body_common
from utils.colors import COLOR_CREAMY_WHITE
from robot_nira.lidar_layout import LOW_RAIL_HEIGHT, SHOULDER_TOP_X
from utils.ocp_utils import show


THICKNESS = body_common.rectangle_slots_height
TAB_WIDTH = body_common.rectangle_slots_width
TAB_DEPTH = body_common.THICKNESS
TAB_PITCH = body_common.rectangle_slots_completed_width / 2
ROW_WIDTH = 2 * TAB_PITCH + TAB_WIDTH
# The rail sits on layer 2 and reaches the underside of layer 3.
RAIL_HEIGHT = body_common.SPACER_LENGTH_TOP


def build_surface(height: float, side: bool = False, front: bool = False,
                  diagonal: bool = False) -> Sketch:
    """Build a profile spanning the outside faces of the two mating layers.

    ``side=True`` selects a swept shoulder with lower legs in both slot rows.
    ``front=True`` selects a low sill without upper tabs; the default is a
    full-height rear plate.
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
        # Drop the nose to a low sill; sweep the side shoulders into the tail.
        deck_top = height - TAB_DEPTH - RAIL_HEIGHT
        sill = deck_top + LOW_RAIL_HEIGHT
        if side:
            Polygon((SHOULDER_TOP_X, height - TAB_DEPTH), (58, sill), (150, sill),
                    (150, height + 1), (SHOULDER_TOP_X, height + 1),
                    align=None, mode=Mode.SUBTRACT)
            # Three slanted gills in each rear shoulder.
            for x in (-49, -37, -25):
                Polygon((x, sill + 5), (x + 4, sill + 5),
                        (x - 2, height - 9), (x - 6, height - 9),
                        align=None, mode=Mode.SUBTRACT)
        elif front:
            with Locations((0, (sill + height + 2) / 2)):
                Rectangle(ROW_WIDTH + 2, height + 2 - sill, mode=Mode.SUBTRACT)
            if diagonal:
                # Narrow the raised front-diagonal sills around the interface
                # cover. Keep all three bottom tabs, with 1 mm of rail over
                # each outer tab's inner end.
                with Locations((-23, TAB_DEPTH + (height + 2) / 2),
                               (23, TAB_DEPTH + (height + 2) / 2)):
                    Rectangle(20, height + 2, mode=Mode.SUBTRACT)
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
    nose = build_model(build_surface(height, front=True))
    side = build_model(build_surface(height, side=True))
    plates = []
    # Local extrusion points towards -Y after the upright rotation. Shift
    # half a thickness so each plate is centred on its slot row.
    upright = Pos(0, THICKNESS / 2, 0) * Rot(X=90)
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


def build_diagonal_plates(z_bottom: float, z_top: float) -> list[Part]:
    """Place low front sill plates and full-height rear diagonal braces."""
    height = z_top + body_common.THICKNESS - z_bottom
    model = build_model(build_surface(height))
    low_model = build_model(build_surface(height, front=True, diagonal=True))
    upright = Pos(0, THICKNESS / 2, 0) * Rot(X=90)
    plates = []
    for location in body_common.diagonal_slots_locations:
        plate = Pos(0, 0, z_bottom) * location * upright * (low_model if location.position.X > 0 else model)
        end = "front" if location.position.X > 0 else "back"
        side = "left" if location.position.Y > 0 else "right"
        plate.label = f"chassis_diagonal_{end}_{side}"
        plate.color = COLOR_CREAMY_WHITE
        plates.append(plate)
    return plates


def main() -> None:
    # Import here so assembly_body can use the builders without a cycle.
    import robot_nira.assembly_body as assembly_body

    height = (3 * body_common.THICKNESS + assembly_body.SPACER_LENGTH_1_to_2
              + assembly_body.SPACER_LENGTH_TOP)
    output = EXPORT_DIR
    output.mkdir(parents=True, exist_ok=True)
    previews = []
    diagonal_height = body_common.SPACER_LENGTH_TOP + 2 * TAB_DEPTH
    for name, is_side, is_front, plate_height, preview_x in (
        ("chassis_back", False, False, height, 0),
        ("chassis_front", False, True, height, 65),
        ("chassis_side", True, False, height, 170),
        ("chassis_diagonal_back", False, False, diagonal_height, 270),
        ("chassis_diagonal_front", False, True, diagonal_height, 330),
    ):
        surface = build_surface(plate_height, side=is_side, front=is_front,
                                diagonal=name.startswith("chassis_diagonal_"))
        model = build_model(surface)
        model.label = name
        export_step(model, str(output / f"{name}.step"))
        drawing = ExportDXF()
        drawing.add_shape(surface)
        drawing.write(str(output / f"{name}.dxf"))
        previews.append(Pos(preview_x, 0, 0) * model)
    show(Compound(children=previews), name="chassis_side", clear=True)


if __name__ == "__main__":
    main()
