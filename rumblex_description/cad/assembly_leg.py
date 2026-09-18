#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, import_step, RigidJoint

from utils.ocp_utils import show
from utils.colors import COLOR_DARK_GRAY

import servo_simplified
import assembly_coxa
import assembly_femur
import assembly_tibia


ANGLE_COXA = 0.0    # degrees
ANGLE_FEMUR = 0.0   # degrees
ANGLE_TIBIA = 0.0   # degrees

ANGLE_COXA %= 360
ANGLE_FEMUR %= 360
ANGLE_TIBIA %= 360

BRACKET_Y_OFFSET = 5.2  # mm


def build_assembly() -> Compound:
    servo = servo_simplified.build_model()
    servo.color = COLOR_DARK_GRAY
    coxa_assembly = assembly_coxa.build_assembly()
    femur_assembly = assembly_femur.build_assembly()
    foot_servo_assembly = assembly_tibia.build_assembly()

    ##########################################################
    ## Connect servo to coxa assembly
    ##########################################################    
    servo.joints["rotation"].connect_to(
        coxa_assembly.joints["body_to_coxa_fixed"],
        angle=ANGLE_COXA,
    )
    servo = Pos(0, BRACKET_Y_OFFSET, 0) * servo

    ##########################################################
    ## Connect coxa assembly to femur assembly
    ##########################################################
    coxa_assembly.joints["coxa_to_femur_fixed"].connect_to(
        femur_assembly.joints["femur_to_coxa_revolute"],
        angle=ANGLE_FEMUR,
    )
    # Rotate around the joint connection point, not the world origin
    p = coxa_assembly.joints["coxa_to_femur_fixed"].location.position
    femur_assembly = Pos(p.X, p.Y, p.Z) * Rot(60, 0, 180) * Pos(p.X + BRACKET_Y_OFFSET, -p.Y, -p.Z) * femur_assembly

    ##########################################################
    ## Connect femur assembly to tibia (foot servo assembly)
    ##########################################################
    femur_assembly.joints["femur_to_tibia_fixed"].connect_to(
        foot_servo_assembly.joints["tibia_to_femur_revolute"],
        angle=ANGLE_TIBIA,
    )
    # Rotate around the joint connection point, not the world origin
    p = femur_assembly.joints["femur_to_tibia_fixed"].location.position
    foot_servo_assembly = Pos(p.X, p.Y, p.Z) * Rot((2*ANGLE_FEMUR) % 360, 0, 180) * Pos(p.X - BRACKET_Y_OFFSET, -p.Y, -p.Z) * foot_servo_assembly



    leg = Compound(children=[servo, coxa_assembly, femur_assembly, foot_servo_assembly])
    return leg

def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_leg.step")

    show(assembly, name="assembly_leg", clear=True)


if __name__ == "__main__":
    main()
