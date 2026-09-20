#!/usr/bin/env python3
"""A leg chain connected at the actual horn mounting faces."""

from pathlib import Path

from build123d import Compound, export_step

import assembly_coxa
import assembly_femur
import assembly_tibia
import servo_simplified
from utils.colors import COLOR_DARK_GRAY
from utils.ocp_utils import show

ANGLE_COXA = 0.0  # degrees
# Display pose: extend the femur outward and turn the foot downward.
ANGLE_FEMUR = 90.0
ANGLE_TIBIA = -90.0


def attach_leg(servo, coxa_angle=0.0, femur_angle=0.0, tibia_angle=0.0) -> Compound:
    """Attach a leg to an already placed servo; angles are in degrees."""
    coxa = assembly_coxa.build_assembly()
    femur = assembly_femur.build_assembly()
    tibia = assembly_tibia.build_assembly()
    servo.joints["rotation"].connect_to(
        coxa.joints["body_to_coxa_fixed"], angle=coxa_angle % 360
    )
    coxa.joints["coxa_to_femur_fixed"].connect_to(
        femur.joints["femur_to_coxa_revolute"], angle=femur_angle % 360
    )
    femur.joints["femur_to_tibia_fixed"].connect_to(
        tibia.joints["tibia_to_femur_revolute"], angle=tibia_angle % 360
    )
    return Compound(children=[coxa, femur, tibia])


def build_assembly() -> Compound:
    servo = servo_simplified.build_model()
    servo.color = COLOR_DARK_GRAY
    leg = attach_leg(servo, ANGLE_COXA, ANGLE_FEMUR, ANGLE_TIBIA)
    return Compound(children=[servo, leg])


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg.step")
    show(assembly, name="assembly_leg", clear=True)


if __name__ == "__main__":
    main()
