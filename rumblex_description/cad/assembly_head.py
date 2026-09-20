#!/usr/bin/env python3

from pathlib import Path

from build123d import (
    BuildPart,
    BuildSketch,
    Compound,
    Cylinder,
    Location,
    Locations,
    Part,
    Plane,
    Pos,
    Rectangle,
    RigidJoint,
    RevoluteJoint,
    Rot,
    export_step,
    export_stl,
    extrude,
    fillet,
    import_step,
)

from utils.ocp_utils import show
from utils.colors import COLOR_DARK_GRAY

import servo_simplified

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
HEAD_POLYGON_RADIUS = 30.0  # mm, circumradius
HEAD_POLYGON_THICKNESS = 20.0  # mm
LIDAR_PLACEHOLDER_RADIUS = 10.0  # mm
LIDAR_PLACEHOLDER_LENGTH = 25.0  # mm
LIDAR_PLACEHOLDER_CENTER_SPACING = 30.0  # mm


def build_head_polygon(side_bracket: Part) -> Part:
    """Build a rounded square head plate on the outer face of the side bracket."""
    bracket_box = side_bracket.bounding_box()
    center_y = (bracket_box.min.Y + bracket_box.max.Y) / 2
    center_z = (bracket_box.min.Z + bracket_box.max.Z) / 2

    # The bracket's outer face is its positive-X face after placement.  The
    # sketch plane is oriented in YZ so extrusion grows away from the servo.
    outer_face = Plane(
        origin=(bracket_box.max.X, center_y, center_z),
        x_dir=(0, 1, 0),
        z_dir=(1, 0, 0),
    )
    with BuildPart() as head_polygon:
        with BuildSketch(outer_face) as sk:
            Rectangle(2 * HEAD_POLYGON_RADIUS, 2 * HEAD_POLYGON_RADIUS)
            fillet(sk.vertices(), radius=HEAD_POLYGON_RADIUS / 4)
        extrude(amount=HEAD_POLYGON_THICKNESS)

    return head_polygon.part


def build_lidar_placeholders(head_polygon: Part) -> Part:
    """Build two Garmin lidar volume placeholders on the polygon's outer face."""
    polygon_box = head_polygon.bounding_box()
    center_y = (polygon_box.min.Y + polygon_box.max.Y) / 2
    center_z = (polygon_box.min.Z + polygon_box.max.Z) / 2

    # The local Z axis is global +X.  Cylinders are centered by default, so
    # offset their center by half their length to place each base on the plate.
    mounting_plane = Plane(
        origin=(polygon_box.max.X + LIDAR_PLACEHOLDER_LENGTH / 2, center_y, center_z),
        x_dir=(0, 1, 0),
        z_dir=(1, 0, 0),
    )
    with BuildPart() as lidar_placeholders:
        with Locations(Location(mounting_plane)):
            with Locations(
                (-LIDAR_PLACEHOLDER_CENTER_SPACING / 2, 0),
                (LIDAR_PLACEHOLDER_CENTER_SPACING / 2, 0),
            ):
                Cylinder(LIDAR_PLACEHOLDER_RADIUS, LIDAR_PLACEHOLDER_LENGTH)

    return lidar_placeholders.part


def build_assembly(include_servo: bool = True) -> Compound:
    """Build the head with an optional visual model of its pitch servo.

    ``servo_mount`` is located at the pitch servo's horn axis.  A parent
    assembly attaches the yaw bracket to this interface.  Set
    ``include_servo=False`` only when that pitch-servo solid is supplied by
    the parent assembly.
    """
    servo = servo_simplified.build_model()
    servo.color = COLOR_DARK_GRAY

    bracket_side_path = Path(__file__).parent / "imported" / "HX-35HM Side Bracket.STEP"
    bracket_side = import_step(str(bracket_side_path))
    bracket_side.color = COLOR_DARK_GRAY

    bracket_side_placed = SIDE_BRACKET_OFFSET * SIDE_BRACKET_ROTATION * bracket_side
    head_polygon = build_head_polygon(bracket_side_placed)
    head_polygon.color = COLOR_DARK_GRAY
    lidar_placeholders = build_lidar_placeholders(head_polygon)
    lidar_placeholders.color = COLOR_DARK_GRAY

    head_children = [bracket_side_placed, head_polygon, lidar_placeholders]
    if include_servo:
        head_children.insert(0, servo)

    head = Compound(children=head_children)
    # Preserve the horn's complete frame, including its Y-axis orientation.
    # ``servo`` is built even when it is not a visible child so this interface
    # remains valid with ``include_servo=False``.
    RigidJoint("servo_mount", head, servo.joints["rotation"].location)
    RevoluteJoint("pitch", head, axis=servo_simplified.HORN_AXIS)
    return head


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_head.step")
    export_stl(assembly, "generated/assembly_head.stl")

    show(assembly, name="assembly_head", clear=True)


if __name__ == "__main__":
    main()
