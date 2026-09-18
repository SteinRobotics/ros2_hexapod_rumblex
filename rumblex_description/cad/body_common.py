#!/usr/bin/env python3

import math
from pathlib import Path
from typing import NamedTuple

from build123d import *
from utils.ocp_utils import show
from servo_cutouts import SERVO_BACK_CUTOUT, SERVO_FRONT_CUTOUT, SERVO_BRACKET_HOLES

THICKNESS = 1.5
OCTAGON_ROTATION = 22.5  


class ServoCutoutConfig(NamedTuple):
    start_index: int
    rotation_deg_clockwise: float
    offset_x: float
    offset_y: float

def shift_points(
    points: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    """Shift 2D points by a constant offset."""
    return [(x + dx, y + dy) for x, y in points]

def rotate_point_sequence(
    points: list[tuple[float, float]], start_index: int
) -> list[tuple[float, float]]:
    """Rotate a point sequence so it starts at start_index."""
    return points[start_index:] + points[:start_index]

def rotate_points_clockwise(
    points: list[tuple[float, float]], degrees: float
) -> list[tuple[float, float]]:
    """Rotate 2D points around the origin by degrees clockwise."""
    radians = math.radians(-degrees)
    cosine = math.cos(radians)
    sine = math.sin(radians)
    return [
        (x * cosine - y * sine, x * sine + y * cosine)
        for x, y in points
    ]


def transform_servo_back_cutout(
    config: ServoCutoutConfig,
) -> list[tuple[float, float]]:
    points = rotate_point_sequence(SERVO_BACK_CUTOUT, config.start_index)
    points = rotate_points_clockwise(points, config.rotation_deg_clockwise)
    return shift_points(points, config.offset_x, config.offset_y)


def transform_servo_front_cutout(
    config: ServoCutoutConfig,
) -> list[tuple[float, float]]:
    points = rotate_point_sequence(SERVO_FRONT_CUTOUT, config.start_index)
    points = rotate_points_clockwise(points, config.rotation_deg_clockwise)
    return shift_points(points, config.offset_x, config.offset_y)


def transform_servo_bracket_holes(
    config: ServoCutoutConfig,
) -> list[tuple[float, float, float]]:
    hole_centers = [(x, y) for x, y, _ in SERVO_BRACKET_HOLES]
    hole_centers = rotate_points_clockwise(hole_centers, config.rotation_deg_clockwise)
    hole_centers = shift_points(hole_centers, config.offset_x, config.offset_y)
    radii = [radius for _, _, radius in SERVO_BRACKET_HOLES]
    return [
        (x, y, radius)
        for (x, y), radius in zip(hole_centers, radii, strict=True)
    ]


octagon_size = 40

# Rechteck-Maße
rect_w = 220
rect_h = 130
half_w = (rect_w / 2) - (octagon_size / 2)
half_h = rect_h / 2 - (octagon_size / 2)
chamfer_length = 20

with BuildSketch() as base_plate:
    Rectangle(rect_w, rect_h)
    chamfer(base_plate.vertices(), chamfer_length)

diag_offset = 5.0
linear_offset = 8.0

octagon_position_left_top = Location((-half_w + diag_offset, half_h - diag_offset), -45.0)
octagon_position_right_top = Location((half_w - diag_offset, half_h - diag_offset), 45.0)
octagon_position_right_bottom = Location((half_w - diag_offset, -half_h + diag_offset), 135.0)
octagon_position_left_bottom = Location((-half_w + diag_offset, -half_h + diag_offset), -135.0)
octagon_position_top_middle = Location((0, half_h + linear_offset), 0.0)
octagon_position_right_middle = Location((half_w + linear_offset, 0), 90.0)
octagon_position_bottom_middle = Location((0, -half_h - linear_offset), 180.0)

# Alle Achtecke haben dieselbe Größe -> reine Positionsliste reicht
octagon_positions = [
    octagon_position_left_top,
    octagon_position_right_top,
    octagon_position_right_bottom,
    octagon_position_left_bottom,
    octagon_position_top_middle,
    octagon_position_right_middle,
    octagon_position_bottom_middle,
]

servo_cutout_diag_offset = 15.5
servo_cutout_linear_offset = 22.0

# Jede Servo-Position: (Achteck-Mittelpunkt, Rotation im Uhrzeigersinn,
# Offset-Richtung in x, Offset-Richtung in y). Bei Diagonalpositionen sind
# dx und dy jeweils +-1, bei linearen Positionen (oben/unten/rechts-mitte)
# ist genau eine der beiden Richtungen 0.
SERVO_POSITIONS = {
    "left_up": (octagon_position_left_top,  -1, 1),
    "right_up": (octagon_position_right_top,  1, 1),
    "right_down": (octagon_position_right_bottom,  1, -1),
    "left_down": (octagon_position_left_bottom,  -1, -1),
    "center_up": (octagon_position_top_middle, 0, 1),
    "head": (octagon_position_right_middle, 1, 0),
    "center_down": (octagon_position_bottom_middle, 0, -1),
}


def _build_servo_cutout_config(
    position: Location, dx_sign: int, dy_sign: int
) -> ServoCutoutConfig:
    is_diagonal = dx_sign != 0 and dy_sign != 0
    offset = servo_cutout_diag_offset if is_diagonal else servo_cutout_linear_offset
    return ServoCutoutConfig(
        start_index=7,
        rotation_deg_clockwise=position.orientation.Z,
        offset_x=position.position.X + dx_sign * offset,
        offset_y=position.position.Y + dy_sign * offset,
    )


SERVO_CUTOUT_CONFIGS = {
    name: _build_servo_cutout_config(*params) for name, params in SERVO_POSITIONS.items()
}

# Je ein dict pro Servo-Position, z. B. SERVO_BACK_CUTOUTS["head"]
SERVO_BACK_CUTOUTS = {
    name: transform_servo_back_cutout(config) for name, config in SERVO_CUTOUT_CONFIGS.items()
}
SERVO_FRONT_CUTOUTS = {
    name: transform_servo_front_cutout(config) for name, config in SERVO_CUTOUT_CONFIGS.items()
}
SERVO_BRACKET_HOLES_BY_POSITION = {
    name: transform_servo_bracket_holes(config) for name, config in SERVO_CUTOUT_CONFIGS.items()
}

LIST_SERVO_BRACKET_HOLES = [
    hole for holes in SERVO_BRACKET_HOLES_BY_POSITION.values() for hole in holes
]

########## Inner sketch -> rectangle + 2x octagons ##########
inner_radius_oct = 54       # Umkreisradius der Achtecke (Hantelköpfe)
inner_distance_oct = 90     # Abstand der Achteck-Mittelpunkte zueinander
inner_rectangle_width = 95  # Höhe des Verbindungsstegs (Griff)

with BuildSketch() as hantel:
    with Locations((-inner_distance_oct / 2, 0), (inner_distance_oct / 2, 0)):
        RegularPolygon(radius=inner_radius_oct, side_count=8, rotation=OCTAGON_ROTATION)
    Rectangle(inner_distance_oct, inner_rectangle_width)


################ Slots ###################

rectangle_slots_width = 8.0
rectangle_slots_height = 1.5
rectangle_slots_completed_width = 4 * rectangle_slots_width
rectangle_slots_distance_border = 10.0

with BuildSketch() as rectangle_slots:
    with Locations((-rectangle_slots_completed_width / 2, 0), (0, 0), (rectangle_slots_completed_width / 2, 0)):
        Rectangle(rectangle_slots_width, rectangle_slots_height)

rectangle_slots_locations = [
    Location((38.0, rect_h / 2 - rectangle_slots_distance_border), 0.0),
    Location((38.0, -rect_h / 2 + rectangle_slots_distance_border), 0.0),
    Location((-38.0, rect_h / 2 - rectangle_slots_distance_border), 0.0),
    Location((-38.0, -rect_h / 2 + rectangle_slots_distance_border), 0.0),
    Location((rect_w / 2 - rectangle_slots_distance_border, 0.0), 90.0),
    Location((-rect_w / 2 + rectangle_slots_distance_border, 0.0), 90.0),
]

############### Holes ###################
hole_radius = 1.5
spacer_outer_radius = 3.0
hole_distance_border = rectangle_slots_distance_border - rectangle_slots_height / 2 - spacer_outer_radius
hole_locations = [
    Location((40.0, rect_h / 2 - hole_distance_border), 0.0),
    Location((-40.0, rect_h / 2 - hole_distance_border), 0.0),
    Location((40.0, -rect_h / 2 + hole_distance_border), 0.0),
    Location((-40.0, -rect_h / 2 + hole_distance_border), 0.0),
    Location((rect_w / 2 - hole_distance_border, 15.0), 0.0),
    Location((rect_w / 2 - hole_distance_border, -15.0), 0.0),
    Location((-rect_w / 2 + hole_distance_border, 15.0), 0.0),
    Location((-rect_w / 2 + hole_distance_border, -15.0), 0.0),
]

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(base_plate)
        for pos in octagon_positions:
            with Locations(pos):
                RegularPolygon(radius=octagon_size, side_count=8, rotation=OCTAGON_ROTATION)

        add(hantel.sketch, mode=Mode.SUBTRACT)

        for loc in rectangle_slots_locations:
            with Locations(loc):
                add(rectangle_slots.sketch, mode=Mode.SUBTRACT)

        for loc in hole_locations:
            with Locations(loc):
                Circle(hole_radius, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/body_layer_common.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/body_layer_common.dxf")

    show(result, name="body_layer_common", clear=True)


if __name__ == "__main__":
    main()