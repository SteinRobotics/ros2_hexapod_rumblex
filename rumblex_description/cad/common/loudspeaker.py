#!/usr/bin/env python3
"""Truncated-cone loudspeaker assembly envelope (millimetres).

The front diameter is 40 mm, the approximate rear diameter is 30 mm, and
the depth is 22 mm. The rear face is at Z=0 and the front face is at Z=22.
Mounting holes, terminals, and diaphragm details are omitted because their
dimensions are unknown.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from build123d import Align, BuildPart, Cone, Location, Part, RigidJoint

from common.export_utils import export_common_part
from utils.colors import COLOR_GREEN
from utils.ocp_utils import show


FRONT_DIAMETER = 40.0
REAR_DIAMETER = 30.0  # Approximate.
DEPTH = 22.0


def build_model() -> Part:
    """Return a tapered loudspeaker envelope with a rear placement joint."""
    with BuildPart() as model:
        Cone(
            bottom_radius=REAR_DIAMETER / 2,
            top_radius=FRONT_DIAMETER / 2,
            height=DEPTH,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        RigidJoint("mount", joint_location=Location((0, 0, 0)))

    result = model.part
    result.label = "loudspeaker"
    result.color = COLOR_GREEN
    return result


def main() -> None:
    result = build_model()
    export_common_part(result, result.label, stl=True)
    show(result, name=result.label, clear=True)


if __name__ == "__main__":
    main()
