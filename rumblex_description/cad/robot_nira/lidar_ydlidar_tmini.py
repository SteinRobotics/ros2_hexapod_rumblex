#!/usr/bin/env python3
"""Simplified YDLIDAR T-mini envelope for robot assemblies (millimetres).

Reference: YDLIDAR T-mini Plus Data Sheet V1.1, Fig. 2:
https://akizukidenshi.com/goodsaffix/YDLIDAR%20T-mini%20Plus%20Data%20Sheet_V1.1%20(240131).pdf

The housing is centred on XY with its mounting face at Z=0. The scanner
axis is +Z. Overall size, scanner diameter, housing height and scan height
follow the drawing; the housing corner radius is an approximation. Mounting
holes, connectors, case taper and rotor details are omitted. This is an
assembly envelope, not a manufacturing model.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from build123d import (
    Align,
    BuildPart,
    BuildSketch,
    Cylinder,
    Location,
    Locations,
    Part,
    RectangleRounded,
    RigidJoint,
    export_step,
    extrude,
)

from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show


BODY_WIDTH = 38.6
BODY_DEPTH = 38.6
BODY_HEIGHT = 21.15
BODY_CORNER_RADIUS = 2.0  # Simplified, not dimensioned in the drawing.
TOTAL_HEIGHT = 33.9
SCANNER_DIAMETER = 36.2
SCANNER_HEIGHT = TOTAL_HEIGHT - BODY_HEIGHT
SCAN_HEIGHT = 26.3


def build_model() -> Part:
    """Return a single solid with mounting-face and scan-centre joints."""
    with BuildPart() as model:
        with BuildSketch():
            RectangleRounded(BODY_WIDTH, BODY_DEPTH, BODY_CORNER_RADIUS)
        extrude(amount=BODY_HEIGHT)

        with Locations((0, 0, BODY_HEIGHT)):
            Cylinder(
                SCANNER_DIAMETER / 2,
                SCANNER_HEIGHT,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )

        RigidJoint("mount", joint_location=Location((0, 0, 0)))
        RigidJoint("scan", joint_location=Location((0, 0, SCAN_HEIGHT)))

    result = model.part
    result.label = "lidar_ydlidar_tmini"
    result.color = COLOR_DARK_GRAY
    return result


def main() -> None:
    result = build_model()
    Path("generated/step").mkdir(parents=True, exist_ok=True)
    export_step(result, "generated/step/lidar_ydlidar_tmini.step")
    show(result, name="lidar_ydlidar_tmini", clear=True)


if __name__ == "__main__":
    main()
