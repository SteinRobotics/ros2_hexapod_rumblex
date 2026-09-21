#!/usr/bin/env python3
"""Faceted, vented cover and sensor platform for Nira's lidar interface.

The open bottom fits over the interface on layer 2. Two M3 mounting ears
retain the cover independently of the board. Dimensions follow the estimated
interface envelope; sensor fastener geometry is not supplied by its model.
"""

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    Align, Box, BuildPart, BuildSketch, Circle, Locations, Mode, Part, Plane,
    Polygon, RectangleRounded, export_step, export_stl, extrude,
)

from robot_nira import EXPORT_DIR, board_ydlidar_tmini_interface as interface
from utils.colors import COLOR_CREAMY_WHITE
from utils.ocp_utils import show

WALL = 1.5
CLEARANCE = 1.0
HEIGHT = interface.TOTAL_HEIGHT + CLEARANCE + WALL
# The board is rotated 90 degrees in the robot; its socket faces +X.
HALF_X = interface.BASE_DEPTH / 2 + CLEARANCE + WALL
HALF_Y = interface.BASE_WIDTH / 2 + CLEARANCE + WALL
MOUNT_Y = HALF_Y + 3.5
MOUNT_HOLE_RADIUS = 1.6
MOUNT_POINTS = ((0, -MOUNT_Y), (0, MOUNT_Y))


def outline(hx: float, hy: float, corner: float) -> list[tuple[float, float]]:
    return [(-hx, -hy + corner), (-hx + corner, -hy),
            (hx - corner, -hy), (hx, -hy + corner),
            (hx, hy - corner), (hx - corner, hy),
            (-hx + corner, hy), (-hx, hy - corner)]


def build_model() -> Part:
    with BuildPart() as cover:
        with BuildSketch():
            Polygon(*outline(HALF_X, HALF_Y, 4), align=None)
        extrude(amount=HEIGHT)
        with BuildSketch():
            Polygon(*outline(HALF_X - WALL, HALF_Y - WALL, 2), align=None)
        extrude(amount=HEIGHT - WALL, mode=Mode.SUBTRACT)

        # Low mounting ears leave the lidar and its optical band unobstructed.
        with BuildSketch():
            with Locations(*MOUNT_POINTS):
                RectangleRounded(12, 10, 2)
        extrude(amount=3)
        with BuildSketch():
            with Locations(*MOUNT_POINTS):
                Circle(MOUNT_HOLE_RADIUS)
        extrude(amount=3, mode=Mode.SUBTRACT)

        # Socket opening through the forward face, open at the bottom for a
        # cable to remain connected while the cover is lifted off.
        with Locations((HALF_X, 0, 0)):
            Box(2 * WALL + 2, interface.CONNECTOR_WIDTH + 4, 8,
                align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        # Angled side gills match the canopy and foot armor. They end above
        # the mounting ears and below the solid sensor-supporting lid.
        for side in (-1, 1):
            plane = Plane(origin=(0, side * (HALF_Y + 1), 0),
                          x_dir=(1, 0, 0), z_dir=(0, -side, 0))
            # Use +Z as the sketch's vertical direction on both faces.
            if side < 0:
                plane = Plane(origin=(0, -HALF_Y - 1, 0),
                              x_dir=(-1, 0, 0), z_dir=(0, 1, 0))
            with BuildSketch(plane):
                for x in (-12, -4, 4, 12):
                    Polygon((x - 2, 4), (x + 1, 4), (x + 3, 9), (x, 9),
                            align=None)
            extrude(amount=WALL + 2, mode=Mode.SUBTRACT)

    cover.part.label = "lidar_interface_housing"
    cover.part.color = COLOR_CREAMY_WHITE
    return cover.part


def main() -> None:
    part = build_model()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(part, str(EXPORT_DIR / "lidar_interface_housing.step"))
    export_stl(part, str(EXPORT_DIR / "lidar_interface_housing.stl"))
    show(part, name=part.label, clear=True)


if __name__ == "__main__":
    main()
