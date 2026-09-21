#!/usr/bin/env python3
"""Simplified OBSBOT Meet SE 1080P 100FPS webcam (millimetres).

Nominal size without mount: 45 mm wide x 36 mm high x 22.2 mm deep.
Reference: https://www.obsbot.com/obsbot-meet-se-full-hd-webcam/specs
Appearance: https://resource-cdn.obsbothk.com/download/obsbot-meet-se/manual/OBSBOTMeetSEUserManual_ENv1.0.pdf

X is width, Y is depth, and Z is up. The bottom face is at Z=0 and
the camera looks along -Y. Corner radii and the lens recess dimensions
and position are visual approximations. The mount joint is a nominal
bottom-centre placement frame, not a measured tripod-hole location.
The monitor clip, cable, ports, threads and small case details are omitted.
This is an assembly envelope, not a manufacturing model.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    Location,
    Mode,
    Part,
    Plane,
    RectangleRounded,
    RigidJoint,
    export_step,
    export_stl,
    extrude,
)

from robot_nira import EXPORT_DIR
from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show


BODY_WIDTH = 45.0
BODY_HEIGHT = 36.0
BODY_DEPTH = 22.2
BODY_CORNER_RADIUS = 5.0  # Estimated.
LENS_RADIUS = 10.0  # Estimated visual detail.
LENS_RECESS_DEPTH = 0.5
LENS_X = -4.0
LENS_HEIGHT = BODY_HEIGHT / 2


def build_model() -> Part:
    """Return a single solid with nominal mount and forward-facing lens joints."""
    # Local sketch Y is global +Z; its normal points towards the viewer (-Y).
    rear_plane = Plane(
        origin=(0, BODY_DEPTH / 2, BODY_HEIGHT / 2),
        x_dir=(1, 0, 0),
        z_dir=(0, -1, 0),
    )
    lens_plane = Plane(
        origin=(LENS_X, -BODY_DEPTH / 2, LENS_HEIGHT),
        x_dir=(1, 0, 0),
        z_dir=(0, -1, 0),
    )
    with BuildPart() as model:
        with BuildSketch(rear_plane):
            RectangleRounded(BODY_WIDTH, BODY_HEIGHT, BODY_CORNER_RADIUS)
        extrude(amount=BODY_DEPTH)

        with BuildSketch(lens_plane):
            Circle(LENS_RADIUS)
        extrude(amount=-LENS_RECESS_DEPTH, mode=Mode.SUBTRACT)

        RigidJoint("mount", joint_location=Location((0, 0, 0)))
        RigidJoint("lens", joint_location=Location(lens_plane))

    result = model.part
    result.label = "webcam_obsbot"
    result.color = COLOR_DARK_GRAY
    return result


def main() -> None:
    result = build_model()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "webcam_obsbot.step"))
    export_stl(result, str(EXPORT_DIR / "webcam_obsbot.stl"))
    show(result, name="webcam_obsbot", clear=True)


if __name__ == "__main__":
    main()
