"""Shared dimensions and geometry for Nira chassis plates."""

from math import hypot

from build123d import (
    BuildPart, BuildSketch, Locations, Part, Rectangle, Sketch, add, extrude,
)

import robot_nira.body_common as body_common
from robot_nira.lidar_layout import LOW_RAIL_HEIGHT, SHOULDER_TOP_X

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


def build_plate_surface(height: float) -> Sketch:
    """Build the tabbed profile shared by rear plates and the front sill."""
    if height <= 2 * TAB_DEPTH:
        raise ValueError("Height must leave room between the upper and lower tabs")

    with BuildSketch() as sketch:
        with Locations((0, height / 2)):
            Rectangle(ROW_WIDTH, height - 2 * TAB_DEPTH)
        for offset in (-TAB_PITCH, 0, TAB_PITCH):
            with Locations((offset, TAB_DEPTH / 2),
                           (offset, height - TAB_DEPTH / 2)):
                Rectangle(TAB_WIDTH, TAB_DEPTH)
    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)
    return model.part
