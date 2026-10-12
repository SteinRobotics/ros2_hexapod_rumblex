#!/usr/bin/env python3
"""Sloped chassis cover with speaker vents and matching side fingers."""

if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from math import ceil, cos, hypot, pi, sin, sqrt
from build123d import (
    Box, BuildPart, BuildSketch, Mode, Part, Plane, Polygon, Pos, insert,
    export_step, extrude,
)
import common.loudspeaker as loudspeaker
import robot_nira.torso_common as torso_common
import robot_nira.lidar_interface_housing as lidar_housing
from robot_nira import EXPORT_DIR
from robot_nira.chassis_common import (
    RAIL_HEIGHT, SLOPE_END_X, SLOPE_START_X, THICKNESS,
    slope_finger_profiles,
)
from robot_nira.lidar_layout import LOW_RAIL_HEIGHT
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show

SPEAKER_Y_POSITIONS = (25.0, -25.0)
SPEAKER_OCTAGON_FLAT_SIZE = 4.0
SPEAKER_SLOT_PITCH = 5.0
SPEAKER_SLOT_FIELD_RADIUS = loudspeaker.FRONT_DIAMETER / 2 - 2.0


def speaker_plane(z_top: float, y: float) -> Plane:
    """Outer slope face at a loudspeaker centre, normal pointing outside."""
    run = SLOPE_END_X - SLOPE_START_X
    drop = RAIL_HEIGHT - LOW_RAIL_HEIGHT
    length = hypot(run, drop)
    return Plane(
        origin=((SLOPE_START_X + SLOPE_END_X) / 2, y, z_top - drop / 2),
        x_dir=(0, 1, 0),
        z_dir=(drop / length, 0, run / length),
    )


def speaker_slot_positions() -> list[tuple[float, float]]:
    """Centred honeycomb rows with every opening inside the solid speaker rim."""
    positions = []
    row_pitch = SPEAKER_SLOT_PITCH * sqrt(3) / 2
    extent = ceil(SPEAKER_SLOT_FIELD_RADIUS / row_pitch)
    for row in range(-extent, extent + 1):
        for column in range(-extent, extent + 1):
            across = (column + 0.5 * (row % 2)) * SPEAKER_SLOT_PITCH
            along = row * row_pitch
            if all(hypot(x, y) <= SPEAKER_SLOT_FIELD_RADIUS
                   for x, y in speaker_octagon_points(across, along)):
                positions.append((across, along))
    return positions


def speaker_octagon_points(across: float, along: float) -> list[tuple[float, float]]:
    """Regular octagonal opening with horizontal and vertical flats."""
    radius = SPEAKER_OCTAGON_FLAT_SIZE / (2 * cos(pi / 8))
    return [(across + radius * cos(pi / 8 + corner * pi / 4),
             along + radius * sin(pi / 8 + corner * pi / 4))
            for corner in range(8)]


def build_slope_cover(z_top: float) -> Part:
    """Bridge the sloped front edges of the two large side plates."""
    run = SLOPE_END_X - SLOPE_START_X
    drop = RAIL_HEIGHT - LOW_RAIL_HEIGHT
    slope_length = hypot(run, drop)
    # Offset the sheet into the chassis, keeping its outer face flush with
    # the shoulder edges and its ends against the inner faces of the sides.
    inward_x = -THICKNESS * drop / slope_length
    inward_z = -THICKNESS * run / slope_length
    side_y = max(
        loc.position.Y for loc in torso_common.rectangle_slots_locations
        if loc.position.X > 0 and abs(loc.orientation.Z) < 1e-6
    )
    inner_y = side_y - THICKNESS / 2
    with BuildPart() as cover:
        with BuildSketch(Plane.XZ):
            Polygon(
                (SLOPE_START_X, z_top),
                (SLOPE_END_X, z_top - drop),
                (SLOPE_END_X + inward_x, z_top - drop + inward_z),
                (SLOPE_START_X + inward_x, z_top + inward_z),
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

    with BuildPart() as perforated:
        insert(result)
        for y in SPEAKER_Y_POSITIONS:
            with BuildSketch(speaker_plane(z_top, y).offset(0.1)):
                for across, along in speaker_slot_positions():
                    Polygon(*speaker_octagon_points(across, along),
                            align=None)
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
    from robot_nira.torso_layout import LAYER_3_BOTTOM

    z_top = LAYER_3_BOTTOM
    model = build_slope_cover(z_top)
    output = EXPORT_DIR / "step"
    output.mkdir(parents=True, exist_ok=True)
    export_step(model, str(output / "chassis_slope_cover.step"))
    show(model, name="chassis_slope_cover", clear=True)


if __name__ == "__main__":
    main()
