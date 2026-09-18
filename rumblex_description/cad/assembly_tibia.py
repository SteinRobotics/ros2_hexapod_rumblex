#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, export_stl, import_step, RevoluteJoint, Axis

from utils.colors import COLOR_DARK_GRAY

import assembly_foot
import servo_simplified

from utils.ocp_utils import show

from servo_simplified import build_model as build_servo

def build_assembly() -> Compound:
    servo = build_servo()
    servo.color = COLOR_DARK_GRAY
    foot_assembly = assembly_foot.build_assembly()

    servo_bb = servo.bounding_box()

    foot_placed = (
        Pos(
            servo_bb.min.X + 12.50,
            servo_bb.max.Y - 1.00,
            servo_bb.max.Z - 30.00,
        )
        * Rot(270, 180, 180)
        * foot_assembly
    )

    foot_servo = Compound(children=[servo, foot_placed])
    RevoluteJoint("tibia_to_femur_revolute", foot_servo, axis=Axis((0, 0, servo_simplified.HORN_Z_CTR), (0, 1, 0)))
    return foot_servo


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_tibia.step")
    export_stl(assembly, "generated/assembly_tibia.stl")
    show(assembly, name="assembly_tibia", clear=True)


if __name__ == "__main__":
    main()
