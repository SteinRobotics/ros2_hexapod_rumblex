#!/usr/bin/env python3
"""Simplified 3S LiPo 11.1 V, 2200 mAh, 35C short-pack battery.

Assumed product: OVONIC short pack with Deans plug. Nominal body dimensions
are 74 x 33 x 26 mm; listed deviations are 5, 2 and 2 mm respectively.
Source: https://www.ovonicshop.com/products/ovonic-3s-2200mah-35c-11-1v-short-lipo-battery-pack-t-plug

X is length, Y is width, and Z is up, with the bottom centred at the origin.
The corner radius is a visual approximation of the soft shrink-wrapped pack.
Cables, connectors, labels and dimensional tolerances are not modelled.
This is a nominal assembly envelope, not a manufacturing or clearance model.
The mount joint is a placement reference, not a physical fastening feature.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildPart,
    Location,
    Part,
    RigidJoint,
    fillet,
)

from common.export_utils import export_common_part
from utils.colors import COLOR_GREEN
from utils.ocp_utils import show


BODY_LENGTH = 74.0
BODY_WIDTH = 33.0
BODY_HEIGHT = 26.0
BODY_CORNER_RADIUS = 2.0  # Estimated visual detail.


def build_model() -> Part:
    """Return the rounded battery body with a bottom-centre placement joint."""
    with BuildPart() as model:
        Box(
            BODY_LENGTH,
            BODY_WIDTH,
            BODY_HEIGHT,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        fillet(model.edges(), radius=BODY_CORNER_RADIUS)
        RigidJoint("mount", joint_location=Location((0, 0, 0)))

    result = model.part
    result.label = "battery_3s_2200mah_35c"
    result.color = COLOR_GREEN
    return result


def main() -> None:
    result = build_model()
    export_common_part(result, result.label, stl=True)
    show(result, name=result.label, clear=True)


if __name__ == "__main__":
    main()
