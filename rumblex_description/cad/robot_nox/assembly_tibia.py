#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nox import EXPORT_DIR

from build123d import Compound, Pos, Rot, export_step, export_stl, RevoluteJoint

from utils.colors import COLOR_DARK_GRAY

import robot_nox.assembly_foot as assembly_foot
import robot_nox.foot_common as foot_common
import common.servo_simplified as servo_simplified

from utils.ocp_utils import show

from common.servo_simplified import build_model as build_servo

# The foot extends along local X, unlike the femur's local Z length axis.
TIBIA_ROLL = 180.0  # degrees about local X

def build_assembly() -> Compound:
    servo = build_servo()
    servo.color = COLOR_DARK_GRAY
    foot_assembly = assembly_foot.build_assembly()

    # The drawing origin is the centre of the mounting-hole pattern.
    # The back plate's inner face rests on the servo's rear case flange.
    foot_placed = (
        Pos(
            0,
            servo_simplified.BODY_Y + servo_simplified.CASE_FLANGE_THICKNESS + foot_common.THICKNESS,
            (servo_simplified.HOLE_Z_LOW + servo_simplified.HOLE_Z_HIGH) / 2,
        )
        * Rot(90, 0, 0)
        * foot_assembly
    )

    # Roll around the midpoint between the horn faces at shaft height.
    # This swaps front and rear horns while keeping them seated in the bracket.
    back_horn_face_y = (
        servo_simplified.BODY_Y
        + servo_simplified.HORN_DISTANCE_TO_BODY
        + servo_simplified.HORN_BACK_Y
    )
    horn_mid_y = (servo_simplified.HORN_FRONT_FACE_Y + back_horn_face_y) / 2
    roll_center = Pos(0, horn_mid_y, servo_simplified.HORN_Z_CTR)
    roll = roll_center * Rot(TIBIA_ROLL, 0, 0) * roll_center.inverse()
    servo = roll * servo
    foot_placed = roll * foot_placed

    foot_servo = Compound(children=[servo, foot_placed])
    # The rear horn now occupies the original front mating face.
    RevoluteJoint("tibia_to_femur_revolute", foot_servo, axis=servo_simplified.HORN_AXIS)
    return foot_servo


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "stl").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / "step/assembly_tibia.step"))
    export_stl(assembly, str(EXPORT_DIR / "stl/assembly_tibia.stl"))
    show(assembly, name="assembly_tibia", clear=True)


if __name__ == "__main__":
    main()
