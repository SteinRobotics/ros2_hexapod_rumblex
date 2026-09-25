#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from robot_nira import EXPORT_DIR

from build123d import Color, Compound, Pos, Rot, export_step, import_step

import robot_nira.assembly_foot as assembly_foot
HX35H_LEG_OFFSET_X = 12.3
HX35H_LEG_OFFSET_Y = -3.5
HX35H_LEG_OFFSET_Z = -30.6
from utils.ocp_utils import show

DARK_GRAY = Color(0.25, 0.25, 0.25)


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).resolve().parents[1] / "imported" / "HX-35H.stp"))
    servo.color = DARK_GRAY
    foot_assembly = assembly_foot.build_assembly()

    servo_bb = servo.bounding_box()

    foot_placed = (
        Pos(
            servo_bb.min.X + HX35H_LEG_OFFSET_X,
            servo_bb.max.Y + HX35H_LEG_OFFSET_Y,
            servo_bb.max.Z + HX35H_LEG_OFFSET_Z,
        )
        * Rot(270, 180, 180)
        * foot_assembly
    )

    return Compound(children=[servo, foot_placed])


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / "step/assembly_foot_servoHX35H.step"))

    show(assembly, name="assembly_foot_servoHX35H", clear=True)


if __name__ == "__main__":
    main()
