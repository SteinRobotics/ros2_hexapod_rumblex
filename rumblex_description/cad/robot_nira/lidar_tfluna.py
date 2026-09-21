#!/usr/bin/env python3
"""Simplified Benewake TF-Luna LiDAR assembly envelope (millimetres).

Nominal overall size: 35 mm wide x 13.5 mm deep x 21.25 mm high.
Reference (dimension drawing on page 2):
https://en.benewake.com/uploadfiles/2024/04/20240426135921367.pdf

X is width, Y is depth, and Z is up. The bottom face is at Z=0 and
the sensor looks along -Y. Side mounting ears have through holes along Y.
The housing outline and ear-to-body transitions are simplified; rear
electronics, connector and cable are omitted. Optical recess depth is an
approximate visual detail. The mount joint
is a nominal bottom-centre frame and the scan joint is at the midpoint
between the optics, not a calibrated ranging origin. This is an assembly
envelope, not a manufacturing model.
"""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Location,
    Locations,
    Mode,
    Part,
    Plane,
    Rectangle,
    RigidJoint,
    export_step,
    export_stl,
    extrude,
)

from robot_nira import EXPORT_DIR
from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show


BODY_WIDTH = 26.5
BODY_DEPTH = 13.5
BODY_HEIGHT = 21.25
OPTIC_DIAMETER = 10.5  # Simplified visual details below.
OPTIC_SPACING = 10.5
OPTIC_HEIGHT = BODY_HEIGHT - 9.0
OPTIC_RECESS_DEPTH = 0.5
MOUNT_HOLE_SPACING = 30.5
MOUNT_HOLE_DIAMETER = 2.2
MOUNT_EAR_RADIUS = 2.25
MOUNT_EAR_THICKNESS = 2.0
MOUNT_HEIGHT = OPTIC_HEIGHT


def build_model() -> Part:
    """Return a single solid with nominal mount and forward scan joints."""
    front_plane = Plane(
        origin=(0, -BODY_DEPTH / 2, OPTIC_HEIGHT),
        x_dir=(1, 0, 0),
        z_dir=(0, -1, 0),
    )
    mounting_plane = Plane(
        origin=(0, BODY_DEPTH / 2, MOUNT_HEIGHT),
        x_dir=(1, 0, 0),
        z_dir=(0, -1, 0),
    )
    with BuildPart() as model:
        Box(
            BODY_WIDTH,
            BODY_DEPTH,
            BODY_HEIGHT,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        with BuildSketch(front_plane):
            with Locations((-OPTIC_SPACING / 2, 0), (OPTIC_SPACING / 2, 0)):
                Circle(OPTIC_DIAMETER / 2)
        extrude(amount=-OPTIC_RECESS_DEPTH, mode=Mode.SUBTRACT)

        # A narrow web joins the rounded ears to the simplified housing.
        with BuildSketch(mounting_plane):
            Rectangle(MOUNT_HOLE_SPACING, 2 * MOUNT_EAR_RADIUS)
            with Locations((-MOUNT_HOLE_SPACING / 2, 0), (MOUNT_HOLE_SPACING / 2, 0)):
                Circle(MOUNT_EAR_RADIUS)
        extrude(amount=MOUNT_EAR_THICKNESS)
        with BuildSketch(mounting_plane):
            with Locations((-MOUNT_HOLE_SPACING / 2, 0), (MOUNT_HOLE_SPACING / 2, 0)):
                Circle(MOUNT_HOLE_DIAMETER / 2)
        extrude(amount=MOUNT_EAR_THICKNESS, mode=Mode.SUBTRACT)

        for name, x in (("mount_left", -MOUNT_HOLE_SPACING / 2),
                        ("mount_right", MOUNT_HOLE_SPACING / 2)):
            RigidJoint(
                name,
                joint_location=Location(Plane(
                    origin=(x, BODY_DEPTH / 2, MOUNT_HEIGHT),
                    x_dir=(1, 0, 0),
                    z_dir=(0, -1, 0),
                )),
            )
        RigidJoint("mount", joint_location=Location((0, 0, 0)))
        RigidJoint("scan", joint_location=Location(front_plane))

    result = model.part
    result.label = "lidar_tfluna"
    result.color = COLOR_DARK_GRAY
    return result


def main() -> None:
    result = build_model()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "lidar_tfluna.step"))
    export_stl(result, str(EXPORT_DIR / "lidar_tfluna.stl"))
    show(result, name="lidar_tfluna", clear=True)


if __name__ == "__main__":
    main()
