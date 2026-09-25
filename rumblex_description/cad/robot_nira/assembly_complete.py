#!/usr/bin/env python3
"""Complete robot, using the same joint interfaces as a single leg."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR

from build123d import Compound
from common.export_utils import export_stl_ignoring_degenerate_faces

import robot_nira.assembly_body_with_servos as assembly_body_with_servos
import robot_nira.assembly_coxa as assembly_coxa
import robot_nira.assembly_head as assembly_head
import robot_nira.assembly_leg as assembly_leg
from utils.ocp_utils import show

ANGLE_LEG_FEMUR = assembly_leg.ANGLE_FEMUR - 90# degrees
ANGLE_LEG_TIBIA = assembly_leg.ANGLE_TIBIA - 90
ANGLE_HEAD_YAW = 0.0
ANGLE_HEAD_PITCH = -60.0


def build_assembly() -> Compound:
    body = assembly_body_with_servos.build_assembly()
    servos = {child.label: child for child in body.children if child.label.startswith("servo_")}
    coxa_head = assembly_coxa.build_assembly()
    servos["servo_head"].joints["rotation"].connect_to(
        coxa_head.joints["body_to_coxa_fixed"], angle=ANGLE_HEAD_YAW % 360
    )
    head = assembly_head.build_assembly()
    coxa_head.joints["coxa_to_femur_fixed"].connect_to(
        head.joints["pitch"], angle=ANGLE_HEAD_PITCH % 360
    )

    legs = []
    for name, servo in servos.items():
        if name == "servo_head":
            continue
        leg = assembly_leg.attach_leg(
            servo, femur_angle=ANGLE_LEG_FEMUR, tibia_angle=ANGLE_LEG_TIBIA
        )
        leg.label = name.replace("servo_", "leg_", 1)
        legs.append(leg)
    return Compound(label="assembly_complete", children=[body, coxa_head, head, *legs])


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "stl").mkdir(parents=True, exist_ok=True)
    # export_step(assembly, str(EXPORT_DIR / "step/assembly_complete.step"))
    export_stl_ignoring_degenerate_faces(assembly, str(EXPORT_DIR / "stl/assembly_complete.stl"))
    show(assembly, name="assembly_complete", clear=True)


if __name__ == "__main__":
    main()
