#!/usr/bin/env python3
"""Low front sills, swept rear shoulders, and a sloped front cover.

Sketch X runs along the slot rows; sketch Y is the assembled height. The
bottom tab tips are at Y=0. All fits are nominal, without kerf compensation.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR
from math import hypot, sqrt

from build123d import (
    Box, BuildPart, BuildSketch, Circle, Compound, ExportDXF, Locations, Part, Plane, Pos,
    Rectangle, Rot, Sketch, Mode, Polygon, add, export_step, extrude,
)

import common.loudspeaker as loudspeaker
import robot_nira.body_common as body_common
import robot_nira.lidar_interface_housing as lidar_housing
from utils.colors import COLOR_CREAMY_WHITE
from robot_nira.lidar_layout import LIDAR_X, LOW_RAIL_HEIGHT, SHOULDER_TOP_X
from utils.ocp_utils import show


THICKNESS = body_common.rectangle_slots_height
SLOPE_END_X = 58.0
COVER_CLEARANCE = 0.5
JOINT_FINGER_COUNT = 5
JOINT_FINGER_LENGTH = 7.0
SPEAKER_Y_POSITIONS = (25.0, -25.0)
SPEAKER_HOLE_RADIUS = 1.5
SPEAKER_HOLE_PITCH = 6.0
SPEAKER_HOLE_FIELD_RADIUS = loudspeaker.FRONT_DIAMETER / 2 - 3.5
TAB_WIDTH = body_common.rectangle_slots_width
TAB_DEPTH = body_common.THICKNESS
TAB_PITCH = body_common.rectangle_slots_completed_width / 2
ROW_WIDTH = 2 * TAB_PITCH + TAB_WIDTH
# The rail sits on layer 2 and reaches the underside of layer 3.
RAIL_HEIGHT = body_common.SPACER_LENGTH_TOP


def slope_finger_profiles(z_top: float) -> list[tuple[tuple[float, float], ...]]:
    """Rectangular box-joint cuts in the sloped XZ edge."""
    run = SLOPE_END_X - SHOULDER_TOP_X
    drop = RAIL_HEIGHT - LOW_RAIL_HEIGHT
    length = hypot(run, drop)
    inward_x = -THICKNESS * drop / length
    inward_z = -THICKNESS * run / length
    pitch = length / (JOINT_FINGER_COUNT + 1)
    profiles = []
    for index in range(1, JOINT_FINGER_COUNT + 1):
        start = (index * pitch - JOINT_FINGER_LENGTH / 2) / length
        end = (index * pitch + JOINT_FINGER_LENGTH / 2) / length
        x0, z0 = SHOULDER_TOP_X + run * start, z_top - drop * start
        x1, z1 = SHOULDER_TOP_X + run * end, z_top - drop * end
        profiles.append(((x0, z0), (x1, z1),
                         (x1 + inward_x, z1 + inward_z),
                         (x0 + inward_x, z0 + inward_z)))
    return profiles


def speaker_plane(z_top: float, y: float) -> Plane:
    """Outer slope face at a loudspeaker centre, normal pointing outside."""
    run = SLOPE_END_X - SHOULDER_TOP_X
    drop = RAIL_HEIGHT - LOW_RAIL_HEIGHT
    length = hypot(run, drop)
    return Plane(
        origin=((SHOULDER_TOP_X + SLOPE_END_X) / 2, y, z_top - drop / 2),
        x_dir=(0, 1, 0),
        z_dir=(drop / length, 0, run / length),
    )


def speaker_hole_positions() -> list[tuple[float, float]]:
    """Staggered hole centres within the front face, leaving a solid rim."""
    positions = []
    row_pitch = SPEAKER_HOLE_PITCH * sqrt(3) / 2
    for row in range(-3, 4):
        for column in range(-3, 4):
            across = (column + 0.5 * (row % 2)) * SPEAKER_HOLE_PITCH
            along = row * row_pitch
            if hypot(across, along) <= SPEAKER_HOLE_FIELD_RADIUS:
                positions.append((across, along))
    return positions


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
            Polygon((SHOULDER_TOP_X, height - TAB_DEPTH), (SLOPE_END_X, sill), (150, sill),
                    (150, height + 1), (SHOULDER_TOP_X, height + 1),
                    align=None, mode=Mode.SUBTRACT)
            for profile in slope_finger_profiles(height - TAB_DEPTH):
                Polygon(*profile, align=None, mode=Mode.SUBTRACT)
            # Three slanted gills in each rear shoulder.
            for x in (-49, -37, -25):
                Polygon((x, sill + 5), (x + 4, sill + 5),
                        (x - 2, height - 9), (x - 6, height - 9),
                        align=None, mode=Mode.SUBTRACT)
        elif front:
            # The straight nose terminates at layer 2. Leave its body below
            # the deck and pass three fingers through the individual slots.
            top_of_body = sill if diagonal else deck_top - TAB_DEPTH
            with Locations((0, (top_of_body + height + 2) / 2)):
                Rectangle(ROW_WIDTH + 2, height + 2 - top_of_body, mode=Mode.SUBTRACT)
            if not diagonal:
                with Locations(*[(offset, deck_top - TAB_DEPTH / 2)
                                 for offset in (-TAB_PITCH, 0, TAB_PITCH)]):
                    Rectangle(TAB_WIDTH, TAB_DEPTH)
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


def build_slope_cover(z_top: float) -> Part:
    """Bridge the sloped front edges of the two large side plates."""
    run = SLOPE_END_X - SHOULDER_TOP_X
    drop = RAIL_HEIGHT - LOW_RAIL_HEIGHT
    slope_length = hypot(run, drop)
    # Offset the sheet into the chassis, keeping its outer face flush with
    # the shoulder edges and its ends against the inner faces of the sides.
    inward_x = -THICKNESS * drop / slope_length
    inward_z = -THICKNESS * run / slope_length
    side_y = max(
        loc.position.Y for loc in body_common.rectangle_slots_locations
        if loc.position.X > 0 and abs(loc.orientation.Z) < 1e-6
    )
    inner_y = side_y - THICKNESS / 2
    with BuildPart() as cover:
        with BuildSketch(Plane.XZ):
            Polygon(
                (SHOULDER_TOP_X, z_top),
                (SLOPE_END_X, z_top - drop),
                (SLOPE_END_X + inward_x, z_top - drop + inward_z),
                (SHOULDER_TOP_X + inward_x, z_top + inward_z),
                align=None,
            )
        extrude(amount=2 * inner_y)
    result = Pos(0, inner_y, 0) * cover.part
    # Alternate short tongues with the side-plate edge. Each tongue fills a
    # matching notch through the side plate thickness, flush at its outside.
    for profile in slope_finger_profiles(z_top):
        with BuildPart() as finger:
            with BuildSketch(Plane.XZ):
                Polygon(*profile, align=None)
            extrude(amount=THICKNESS + 0.05)
        result = result + Pos(0, side_y + THICKNESS / 2, 0) * finger.part
        result = result + Pos(0, -inner_y + 0.05, 0) * finger.part
    # The lidar housing reaches the lower centre of the slope. Its rear wall
    # occupies only the middle of the cover, so trim that area locally.
    housing_rear_x = LIDAR_X - lidar_housing.HALF_X
    housing_top_z = z_top - RAIL_HEIGHT + lidar_housing.HEIGHT
    notch = Pos((housing_rear_x + SLOPE_END_X) / 2, 0, housing_top_z - 50) * Box(
        SLOPE_END_X - housing_rear_x + 2 * COVER_CLEARANCE,
        2 * (lidar_housing.HALF_Y + COVER_CLEARANCE),
        100,
    )
    result = result - notch
    with BuildPart() as perforated:
        add(result)
        for y in SPEAKER_Y_POSITIONS:
            with BuildSketch(speaker_plane(z_top, y).offset(0.1)):
                with Locations(*speaker_hole_positions()):
                    Circle(SPEAKER_HOLE_RADIUS)
            extrude(amount=-THICKNESS - 0.2, mode=Mode.SUBTRACT)
    result = perforated.part
    result.label = "chassis_slope_cover"
    result.color = COLOR_CREAMY_WHITE
    return result


def build_speakers(z_top: float) -> list[Part]:
    """Seat two speaker fronts against the inner face of the sloped cover."""
    model = loudspeaker.build_model()
    speakers = []
    for name, y in (("left", SPEAKER_Y_POSITIONS[0]),
                    ("right", SPEAKER_Y_POSITIONS[1])):
        speaker = (speaker_plane(z_top, y).location
                   * Pos(0, 0, -THICKNESS - loudspeaker.DEPTH) * model)
        speaker.label = f"loudspeaker_{name}"
        speakers.append(speaker)
    return speakers


def main() -> None:
    # Import here so assembly_body can use the builders without a cycle.
    import robot_nira.assembly_body as assembly_body

    height = (3 * body_common.THICKNESS + assembly_body.SPACER_LENGTH_1_to_2
              + assembly_body.SPACER_LENGTH_TOP)
    output = EXPORT_DIR
    output.mkdir(parents=True, exist_ok=True)
    (output / "step").mkdir(parents=True, exist_ok=True)
    (output / "dxf").mkdir(parents=True, exist_ok=True)
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
        export_step(model, str(output / "step" / f"{name}.step"))
        drawing = ExportDXF()
        drawing.add_shape(surface)
        drawing.write(str(output / "dxf" / f"{name}.dxf"))
        previews.append(Pos(preview_x, 0, 0) * model)
    slope_cover = build_slope_cover(assembly_body.SPACER_LENGTH_0_to_1
                                    + assembly_body.SPACER_LENGTH_1_to_2
                                    + 3 * body_common.THICKNESS
                                    + assembly_body.SPACER_LENGTH_TOP)
    export_step(slope_cover, str(output / "step/chassis_slope_cover.step"))
    previews.append(Pos(440, 0, 0) * slope_cover)
    show(Compound(children=previews), name="chassis_side", clear=True)


if __name__ == "__main__":
    main()
