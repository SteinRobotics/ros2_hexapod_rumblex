#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from robot_nira import EXPORT_DIR

from build123d import (
    BuildPart, BuildSketch, Circle, Compound, Locations, Mode,
    Part, Plane, Polygon, Pos, Rectangle, RectangleRounded, RigidJoint,
    RevoluteJoint, Rot, export_step, export_stl, extrude, import_step,
)

from utils.ocp_utils import show
from utils.colors import COLOR_CREAMY_WHITE, COLOR_DARK_GRAY, COLOR_WINE_RED
from utils import spacer
from robot_nira import webcam_obsbot
import common.servo_simplified as servo_simplified

# The outer side-bracket mounting pair is 24.45 mm apart, matching one
# vertical pair of servo M2 holes.  This placement maps that pair onto the
# servo's front (-Y) face at X = +HOLE_X.
SIDE_BRACKET_ROTATION = Rot(0, 0, 90)
# Vendor STEP registration offset, retained until its mounting frame is measured.
SIDE_BRACKET_OFFSET = Pos(
    29.999,
    -44.3795 + servo_simplified.BODY_Y + servo_simplified.CASE_FLANGE_THICKNESS,
    -165.2392,
)
# Pod coordinates: X across the face, -Y forward, Z up, as in the webcam.
# The frame's origin is on the adapter's rear face, aligned to the bracket centre.
PLATE_THICKNESS = 3.0
BRACKET_SPACER_LENGTH = 6.0
SPACER_OUTER_DIAMETER = 5.0
M3_CLEARANCE = 3.2
# Measured from the four diameter-3.2 circular edges of the vendor STEP.
BRACKET_HOLE_HALF_PITCH = 4.9497474683
BRACKET_CENTER_Y = 14.5037836201
BRACKET_CENTER_Z = 14.7999609333
CAMERA_Y = -28.0
CAMERA_BASE_Z = -14.0
CHIN_Z = CAMERA_BASE_Z - PLATE_THICKNESS
BROW_Z = CAMERA_BASE_Z + webcam_obsbot.BODY_HEIGHT + 2.0
CAGE_SPACER_LENGTH = BROW_Z - CAMERA_BASE_Z
CAGE_HOLES = [(x, y) for x in (-28.0, 28.0) for y in (-14.0, -29.0)]
TAB_CENTERS = (-18.0, 18.0)
TAB_WIDTH = 8.0
# Swept shoulders and a narrow, chamfered nose echo Nira's body armor.
DECK_OUTLINE = [
    (-25, -3), (-34, -12), (-34, -32), (-21, -39),
    (21, -39), (34, -32), (34, -12), (25, -3),
]


def build_deck(*, brow: bool = False) -> Part:
    """Flat, tabbed chin/brow plate with common M3 standoff holes.

    The chin has a 6.6 mm tripod screw clearance bore. Socket position and
    insertion depth remain the estimates documented by webcam_obsbot;
    select the actual screw length after measuring the camera.
    """
    with BuildPart() as plate:
        with BuildSketch():
            Polygon(*DECK_OUTLINE, align=None)
            with Locations(*[(x, -PLATE_THICKNESS / 2) for x in TAB_CENTERS]):
                Rectangle(TAB_WIDTH, PLATE_THICKNESS)
            with Locations(*CAGE_HOLES):
                Circle(M3_CLEARANCE / 2, mode=Mode.SUBTRACT)
            if brow:
                # Three swept cooling slots, behind the lens plane.
                for x in (-12, 0, 12):
                    with Locations((x, -20)):
                        RectangleRounded(4, 16, 1.5, rotation=-20,
                                         mode=Mode.SUBTRACT)
            else:
                with Locations((webcam_obsbot.MOUNT_HOLE_X,
                                CAMERA_Y + webcam_obsbot.MOUNT_HOLE_Y)):
                    Circle(3.3, mode=Mode.SUBTRACT)
            # Open-backed cable notch between the structural tabs.
            with Locations((0, -4)):
                RectangleRounded(14, 12, 2, mode=Mode.SUBTRACT)
        extrude(amount=PLATE_THICKNESS)
    plate.part.label = "head_brow" if brow else "head_chin"
    plate.part.color = COLOR_CREAMY_WHITE
    return plate.part


def build_adapter() -> Part:
    """Vented rear plate: four bracket holes and slots for both deck tabs.

    Flat plates and cylindrical spacers use nominal fits, without machining
    or kerf allowances. The deck tabs locate the plates; the four long M3
    standoffs clamp the cage together. Fasteners are omitted, like the body.
    """
    rear_plane = Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    with BuildPart() as plate:
        with BuildSketch(rear_plane):
            Polygon((-24, CHIN_Z - 4), (24, CHIN_Z - 4), (29, -10),
                    (29, BROW_Z - 5), (22, BROW_Z + 7),
                    (-22, BROW_Z + 7), (-29, BROW_Z - 5), (-29, -10),
                    align=None)
            with Locations(*[(x, z) for x in (-BRACKET_HOLE_HALF_PITCH,
                                              BRACKET_HOLE_HALF_PITCH)
                             for z in (-BRACKET_HOLE_HALF_PITCH,
                                       BRACKET_HOLE_HALF_PITCH)]):
                Circle(M3_CLEARANCE / 2, mode=Mode.SUBTRACT)
            for z in (CHIN_Z, BROW_Z):
                with Locations(*[(x, z + PLATE_THICKNESS / 2) for x in TAB_CENTERS]):
                    Rectangle(TAB_WIDTH, PLATE_THICKNESS, mode=Mode.SUBTRACT)
            # High central cable window and paired cheek gills.
            with Locations((0, 17)):
                RectangleRounded(15, 10, 2, mode=Mode.SUBTRACT)
            for x in (-21, 21):
                for z in (-6, 2, 10):
                    with Locations((x, z)):
                        RectangleRounded(9, 3, 1, mode=Mode.SUBTRACT)
        extrude(amount=PLATE_THICKNESS)
    plate.part.label = "head_rear_adapter"
    plate.part.color = COLOR_CREAMY_WHITE
    return plate.part


def build_camera_pod(side_bracket: Part) -> list[Part]:
    """Place the cage outboard of the bracket, looking along servo-local +X."""
    frame = Pos(side_bracket.bounding_box().max.X + BRACKET_SPACER_LENGTH,
                BRACKET_CENTER_Y, BRACKET_CENTER_Z) * Rot(Z=90)
    camera = webcam_obsbot.build_model()
    camera_mount = Pos(webcam_obsbot.MOUNT_HOLE_X,
                       CAMERA_Y + webcam_obsbot.MOUNT_HOLE_Y, CAMERA_BASE_Z)
    camera = camera_mount * camera.joints["mount"].location.inverse() * camera
    camera.label = "webcam_obsbot"
    parts = [build_adapter(), Pos(Z=CHIN_Z) * build_deck(),
             Pos(Z=BROW_Z) * build_deck(brow=True), camera]
    for i, (x, y) in enumerate(CAGE_HOLES):
        post = Pos(x, y, CAMERA_BASE_Z + CAGE_SPACER_LENGTH / 2) * spacer.build_model(
            outer_diameter=SPACER_OUTER_DIAMETER,
            inner_diameter=M3_CLEARANCE, length=CAGE_SPACER_LENGTH)
        post.label = f"head_cage_spacer_{i}"
        post.color = COLOR_WINE_RED
        parts.append(post)
    for i, (x, z) in enumerate((x, z)
                              for x in (-BRACKET_HOLE_HALF_PITCH, BRACKET_HOLE_HALF_PITCH)
                              for z in (-BRACKET_HOLE_HALF_PITCH, BRACKET_HOLE_HALF_PITCH)):
        post = Pos(x, BRACKET_SPACER_LENGTH / 2, z) * Rot(X=90) * spacer.build_model(
            outer_diameter=SPACER_OUTER_DIAMETER,
            inner_diameter=M3_CLEARANCE, length=BRACKET_SPACER_LENGTH)
        post.label = f"head_bracket_spacer_{i}"
        post.color = COLOR_WINE_RED
        parts.append(post)
    return [frame * part for part in parts]


def build_assembly(include_servo: bool = True) -> Compound:
    """Build the head with an optional visual model of its pitch servo.

    ``servo_mount`` is located at the pitch servo's horn axis.  A parent
    assembly attaches the yaw bracket to this interface.  Set
    ``include_servo=False`` only when that pitch-servo solid is supplied by
    the parent assembly.
    """
    servo = servo_simplified.build_model()
    servo.color = COLOR_DARK_GRAY

    bracket_side_path = Path(__file__).resolve().parents[1] / "imported" / "HX-35HM Side Bracket.STEP"
    bracket_side = import_step(str(bracket_side_path))
    bracket_side.color = COLOR_DARK_GRAY

    bracket_side_placed = SIDE_BRACKET_OFFSET * SIDE_BRACKET_ROTATION * bracket_side
    bracket_side_placed.label = "head_side_bracket"
    servo.label = "head_pitch_servo"
    head_children = [bracket_side_placed, *build_camera_pod(bracket_side_placed)]
    if include_servo:
        head_children.insert(0, servo)

    head = Compound(label="assembly_head", children=head_children)
    # Preserve the horn's complete frame, including its Y-axis orientation.
    # ``servo`` is built even when it is not a visible child so this interface
    # remains valid with ``include_servo=False``.
    RigidJoint("servo_mount", head, servo.joints["rotation"].location)
    RevoluteJoint("pitch", head, axis=servo_simplified.HORN_AXIS)
    return head


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / "assembly_head.step"))
    export_stl(assembly, str(EXPORT_DIR / "assembly_head.stl"))

    show(assembly, name="assembly_head", clear=True)


if __name__ == "__main__":
    main()
