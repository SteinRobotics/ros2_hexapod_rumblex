#!/usr/bin/env python3
"""Sloped chassis cover with speaker vents and matching side fingers."""

if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from math import hypot
from build123d import (
    Box, BuildPart, BuildSketch, Mode, Part, Plane, Polygon, Pos, add,
    export_step, extrude,
)
import common.loudspeaker as loudspeaker
import robot_nira.body_common as body_common
import robot_nira.lidar_interface_housing as lidar_housing
from robot_nira import EXPORT_DIR
from robot_nira.armor_style import GILL_SWEEP_RATIO, GILL_WIDTH_RATIO, gill_points
from robot_nira.chassis_common import (
    RAIL_HEIGHT, SLOPE_END_X, THICKNESS, slope_finger_profiles,
)
from robot_nira.lidar_layout import LIDAR_X, LOW_RAIL_HEIGHT, SHOULDER_TOP_X
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show

COVER_CLEARANCE = 0.5
SPEAKER_Y_POSITIONS = (25.0, -25.0)
SPEAKER_GILL_SPAN = 3.5
SPEAKER_GILL_ROW_GAP = 1.0
SPEAKER_SLOT_PITCH = 6.0
SPEAKER_SLOT_FIELD_RADIUS = loudspeaker.FRONT_DIAMETER / 2 - 3.5


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


def speaker_slot_positions() -> list[tuple[float, float]]:
    """Staggered gill centres within the speaker face, leaving a solid rim."""
    positions = []
    # A gill spans width + sweep along the slope. Leave a full bridge between rows.
    row_pitch = (SPEAKER_GILL_SPAN * (GILL_WIDTH_RATIO + GILL_SWEEP_RATIO)
                 + SPEAKER_GILL_ROW_GAP)
    for row in range(-3, 4):
        for column in range(-3, 4):
            across = (column + 0.5 * (row % 2)) * SPEAKER_SLOT_PITCH
            along = row * row_pitch
            if hypot(across, along) <= SPEAKER_SLOT_FIELD_RADIUS:
                positions.append((across, along))
    return positions


def speaker_gill_points(across: float, along: float, side: int):
    """Centre and mirror a foot-style gill on a speaker face."""
    span = SPEAKER_GILL_SPAN
    width = span * GILL_WIDTH_RATIO
    sweep = span * GILL_SWEEP_RATIO
    points = gill_points(along + (sweep - width) / 2,
                         side * across - span / 2, span, side=side)
    return [(transverse, longitudinal) for longitudinal, transverse in points]


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
                for across, along in speaker_slot_positions():
                    Polygon(*speaker_gill_points(across, along, -1 if y > 0 else 1),
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
    import robot_nira.assembly_body as assembly_body

    z_top = (assembly_body.SPACER_LENGTH_0_to_1
             + assembly_body.SPACER_LENGTH_1_to_2
             + 3 * body_common.THICKNESS
             + assembly_body.SPACER_LENGTH_TOP)
    model = build_slope_cover(z_top)
    output = EXPORT_DIR / "step"
    output.mkdir(parents=True, exist_ok=True)
    export_step(model, str(output / "chassis_slope_cover.step"))
    show(model, name="chassis_slope_cover", clear=True)


if __name__ == "__main__":
    main()
