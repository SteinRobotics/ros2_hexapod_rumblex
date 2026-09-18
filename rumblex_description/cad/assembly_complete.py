#!/usr/bin/env python3
"""Complete robot assembly with named placement steps for each interface."""

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step

import assembly_coxa
import assembly_femur
import assembly_tibia
import assembly_head
import assembly_body_with_servos
import bracket_inclined
import body_common
from utils.ocp_utils import show

ANGLE_LEG_FEMUR = 0.0  # degrees
ANGLE_LEG_TIBIA = 0.0  # degrees
ANGLE_HEAD_YAW = 0.0  # degrees
ANGLE_HEAD_PITCH = 0.0  # degrees

BRACKET_Y_OFFSET = 5.2  # mm

# Align the head servo frame with the coxa-to-femur bracket frame.  Their
# axes differ by one quarter-turn about X and Y.  The inclined bracket adds
# its physical slope to the Y alignment.
HEAD_MOUNT_ROTATION_X = 0.0  # degrees
HEAD_MOUNT_ROTATION_Y = 270.0 + bracket_inclined.THETA_DEG  # degrees
HEAD_MOUNT_ROTATION_Z = 0.0  # degrees

# This clearance is applied in the head's local frame before the rotation
# above.  It was previously embedded in the final Pos(...) expression.
HEAD_MOUNT_LOCAL_X_OFFSET = 0.0
HEAD_MOUNT_LOCAL_Y_OFFSET = -BRACKET_Y_OFFSET
HEAD_MOUNT_LOCAL_Z_OFFSET = 0.0

# All 7 body servo positions minus "head", which is not a leg attachment.
LEG_POSITIONS = [k for k in body_common.SERVO_CUTOUT_CONFIGS if k != "head"]

# Keep leg placement tied to the servo placement defined by the body assembly.
# The coxa shaft is offset from the servo origin by the bracket clearance.
_LEG_SHAFT_Z = assembly_body_with_servos.SERVO_Z + BRACKET_Y_OFFSET


def _align_coxa_with_body_servo(coxa: Compound) -> Compound:
    """Apply the coordinate-frame conversion from a body servo to a coxa."""
    position = coxa.joints["body_to_coxa_fixed"].location.position
    return (
        Pos(position.X, position.Y, position.Z)
        * Rot(0, 180, 180)
        * Pos(
            -position.X,
            -position.Y + BRACKET_Y_OFFSET + assembly_body_with_servos.SERVO_Z_MID,
            -position.Z - BRACKET_Y_OFFSET,
        )
        * coxa
    )


def _attach_femur(coxa: Compound, femur: Compound) -> Compound:
    """Connect and orient the femur at the coxa's outer bracket."""
    coxa.joints["coxa_to_femur_fixed"].connect_to(
        femur.joints["femur_to_coxa_revolute"], angle=ANGLE_LEG_FEMUR
    )
    position = coxa.joints["coxa_to_femur_fixed"].location.position
    return (
        Pos(position.X, position.Y, position.Z)
        * Rot(60, 0, 180)
        * Pos(position.X + BRACKET_Y_OFFSET, -position.Y, -position.Z)
        * femur
    )


def _attach_tibia(femur: Compound, tibia: Compound) -> Compound:
    """Connect and orient the tibia at the femur's outer bracket."""
    femur.joints["femur_to_tibia_fixed"].connect_to(
        tibia.joints["tibia_to_femur_revolute"], angle=ANGLE_LEG_TIBIA
    )
    position = femur.joints["femur_to_tibia_fixed"].location.position
    return (
        Pos(position.X, position.Y, position.Z)
        * Rot((2 * ANGLE_LEG_FEMUR) % 360, 0, 180)
        * Pos(position.X - BRACKET_Y_OFFSET, -position.Y, -position.Z)
        * tibia
    )


def _build_leg() -> Compound:
    """Build one leg in the coordinate frame of its coxa servo."""
    coxa = _align_coxa_with_body_servo(assembly_coxa.build_assembly())
    femur = _attach_femur(coxa, assembly_femur.build_assembly())
    tibia = _attach_tibia(femur, assembly_tibia.build_assembly())
    return Compound(children=[coxa, femur, tibia])


def _place_leg_at_body_cutout(name: str) -> Compound:
    """Place one reusable leg chain at a named body servo cutout."""
    config = body_common.SERVO_CUTOUT_CONFIGS[name]
    leg = assembly_body_with_servos.servo_location(config, _LEG_SHAFT_Z) * _build_leg()
    leg.label = f"leg_{name}"
    return leg


def _build_head() -> tuple[Compound, Compound]:
    """Build the yaw bracket driven by the body and its pitch-mounted head."""
    coxa = assembly_coxa.build_assembly()
    body_servo = assembly_body_with_servos.servo_by_name["servo_head"]
    body_servo.joints["rotation"].connect_to(
        coxa.joints["body_to_coxa_fixed"], angle=ANGLE_HEAD_YAW
    )
    position = coxa.joints["body_to_coxa_fixed"].location.position
    coxa = (
        Pos(position.X, position.Y, position.Z)
        * Rot(180, 180, 180)
        * Pos(-position.X, -position.Y, -position.Z + BRACKET_Y_OFFSET)
        * coxa
    )

    head = assembly_head.build_assembly()
    coxa.joints["coxa_to_femur_fixed"].connect_to(head.joints["servo_mount"])
    mount_position = head.joints["servo_mount"].location.position

    # Read the placement from right to left:
    # 1. move the servo horn to the origin, including bracket clearance;
    # 2. align the head's local frame to the pitch bracket;
    # 3. put the horn back at its joint location.
    #
    # ``ANGLE_HEAD_PITCH`` is intentionally the Z Euler component here.  The
    # fixed X/Y calibration maps that local rotation onto the physical pitch
    # direction of the assembled head.
    head = (
        Pos(mount_position.X, mount_position.Y, mount_position.Z)
        * Rot(
            HEAD_MOUNT_ROTATION_X,
            HEAD_MOUNT_ROTATION_Y,
            HEAD_MOUNT_ROTATION_Z + ANGLE_HEAD_PITCH,
        )
        * Pos(
            -mount_position.X + HEAD_MOUNT_LOCAL_X_OFFSET,
            -mount_position.Y + HEAD_MOUNT_LOCAL_Y_OFFSET,
            -mount_position.Z + HEAD_MOUNT_LOCAL_Z_OFFSET,
        )
        * head
    )
    return coxa, head


def build_assembly() -> Compound:
    body = assembly_body_with_servos.build_assembly()
    coxa_head, head = _build_head()
    leg_instances = [_place_leg_at_body_cutout(name) for name in LEG_POSITIONS]

    return Compound(
        label="assembly_complete",
        children=[body, coxa_head, head, *leg_instances],
    )


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_complete.step")
    show(assembly, name="assembly_complete", clear=True)


if __name__ == "__main__":
    main()
