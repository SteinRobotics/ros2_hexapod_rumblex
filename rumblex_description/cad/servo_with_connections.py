#!/usr/bin/env python3

from pathlib import Path

from build123d import *
from utils.ocp_utils import show

from utils.colors import (
    COLOR_DARK_GRAY,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_GREEN,
    COLOR_BLUE,
)

# Marker geometry — adjust to match your model's units/scale
MARKER_RADIUS = 0.5
MARKER_LENGTH = 2.0

# All positions are given as (x, y, z) tuples. Each group's markers are drawn
# with their cylinder axis along Y, since these positions come from a
# top-view / back-view convention where Y is the viewing direction.

SERVO_CONNECTIONS_TOP_VIEW = {
    "right_down": (10.25, -16.5, 27.2),
    "left_down": (-10.25, -16.5, 27.2),
    "right_up":  ( 10.25, -16.5,  2.2),
    "left_up":   (-10.25, -16.5,  2.2),
}

SERVO_CONNECTIONS_BACK_VIEW = {
    "right_down": (10.25, 17.5, 27.2),
    "left_down": (-10.25, 17.5, 27.2),
    "right_up":  ( 10.25, 17.5,  2.2),
    "left_up":   (-10.25, 17.5,  2.2),
}


# 27.05 - 2.60 = 24.45, which is the distance between the two servo connection holes along Z.
# 24.45 / 2 = 12.225, which is the distance from the center of the servo to each connection hole along Z. 
SERVO_CONNECTIONS_FLAT = [
    (10.25, 12.5),
    (-10.25, 12.5),
    (10.25,  -12.5),
    (-10.25,  -12.5),
]

SERVO_HORN_TOP_VIEW = {
    "center": (0.0, -20.5, 35.35),
    "right":  (7.0, -20.5, 35.35),
    "left":  (-7.0, -20.5, 35.35),
    "up":    (-0.0, -20.5, 42.35),
    "down":   (0.0, -20.5, 28.35),
}

SERVO_HORN_BACK_VIEW = {
    "center": (0.0, 22.0, 35.35),
    "right":  (7.0, 22.0, 35.35),
    "left":  (-7.0, 22.0, 35.35),
    "up":    (-0.0, 22.0, 42.35),
    "down":   (0.0, 22.0, 28.35),
}

# Group name -> (positions dict, marker color)
MARKER_GROUPS = {
    "connections_top": (SERVO_CONNECTIONS_TOP_VIEW, COLOR_RED),
    "connections_back": (SERVO_CONNECTIONS_BACK_VIEW, COLOR_ORANGE),
    "servo_horn_top": (SERVO_HORN_TOP_VIEW, COLOR_GREEN),
    "servo_horn_back": (SERVO_HORN_BACK_VIEW, COLOR_BLUE),
}


def marker_location(position: tuple[float, float, float]) -> Location:
    """Location with Z aligned along global Y, so the marker cylinder's
    axis points along Y at the given position."""
    return Location(Plane(origin=position, z_dir=(0, 1, 0)))


def build_markers() -> list[Part]:
    """Build one small labeled Part per named position, colored by group."""
    marker_parts = []

    for group_name, (positions, color) in MARKER_GROUPS.items():
        for label, pos in positions.items():
            with BuildPart() as marker:
                with Locations(marker_location(pos)):
                    Cylinder(radius=MARKER_RADIUS, height=MARKER_LENGTH)
            marker.part.color = color
            marker.part.label = f"{group_name}:{label}"
            marker_parts.append(marker.part)

    return marker_parts


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).parent / "imported" / "HX-35H.stp"))
    servo.color = COLOR_DARK_GRAY

    marker_parts = build_markers()

    return Compound(children=[servo, *marker_parts])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/servoHX-35H_connection_markers.step")

    show(assembly, name="servoHX-35H_connection_markers", clear=True)


if __name__ == "__main__":
    main()