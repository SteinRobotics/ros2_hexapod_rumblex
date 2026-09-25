"""Shared dimensions, profiles, and placement for Nira chassis plates."""

from math import hypot

from build123d import (
    BuildPart, BuildSketch, Locations, Mode, Part, Polygon, Pos, Rectangle,
    Rot, Sketch, add, extrude,
)

import robot_nira.body_common as body_common
from robot_nira.lidar_layout import LOW_RAIL_HEIGHT, SHOULDER_TOP_X
from utils.colors import COLOR_CREAMY_WHITE

THICKNESS = body_common.rectangle_slots_height
SLOPE_END_X = 58.0
JOINT_FINGER_COUNT = 5
JOINT_FINGER_LENGTH = 7.0
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


def build_plate_surface(height: float, *, side: bool = False, front: bool = False) -> Sketch:
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
            top_of_body = deck_top - TAB_DEPTH
            with Locations((0, (top_of_body + height + 2) / 2)):
                Rectangle(ROW_WIDTH + 2, height + 2 - top_of_body, mode=Mode.SUBTRACT)
            with Locations(*[(offset, deck_top - TAB_DEPTH / 2)
                             for offset in (-TAB_PITCH, 0, TAB_PITCH)]):
                Rectangle(TAB_WIDTH, TAB_DEPTH)
    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)
    return model.part


def build_plates(z_bottom: float, z_top: float) -> list[Part]:
    """Place front, back, and side plates in the shared slots."""
    from robot_nira import chassis_back, chassis_front, chassis_side

    height = z_top + body_common.THICKNESS - z_bottom
    end = chassis_back.build_model(height)
    nose = chassis_front.build_model(height)
    side = chassis_side.build_model(height)
    plates = []
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
    """Place rear and front diagonal braces in their slots."""
    from robot_nira import chassis_diagonal_back, chassis_diagonal_front

    height = z_top + body_common.THICKNESS - z_bottom
    rear = chassis_diagonal_back.build_model(height)
    front = chassis_diagonal_front.build_model(height)
    upright = Pos(0, THICKNESS / 2, 0) * Rot(X=90)
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
