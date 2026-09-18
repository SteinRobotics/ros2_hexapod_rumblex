#!/usr/bin/env python3

from pathlib import Path

from build123d import Compound, Pos, Rot, export_step, export_stl, import_step, RevoluteJoint, RigidJoint, Axis


from utils.colors import COLOR_DARK_GRAY
import servo_simplified
import bracket_inclined
from utils.ocp_utils import show

BRAKET_BOTTOM_Z_OFFSET = 5.65  # mm
BRAKET_INCLINED_X_OFFSET = -4.5  # mm

def _center_xy_below(part, reference_bb):
    """Translate part so its XY is centred under reference_bb and its top (Z_max) touches reference_bb.min.Z."""
    bb = part.bounding_box()
    return Pos(
        (reference_bb.min.X + reference_bb.max.X) / 2 - (bb.min.X + bb.max.X) / 2,
        (reference_bb.min.Y + reference_bb.max.Y) / 2 - (bb.min.Y + bb.max.Y) / 2,
        reference_bb.min.Z - bb.max.Z,
    ) * part


def build_assembly() -> Compound:
    servo = servo_simplified.build_model()
    servo.color = COLOR_DARK_GRAY

    bracket_botton = import_step(str(Path(__file__).parent / "imported" / "HX-35HM Botton Bracket.STEP"))
    bracket_botton.color = COLOR_DARK_GRAY

    bracket_inclinded_u_shape = bracket_inclined.build_bracket()
    bracket_inclinded_u_shape.color = COLOR_DARK_GRAY

    servo_bb = servo.bounding_box()
    bracket_botton_placed = _center_xy_below(Rot(180, 0, 90) * bracket_botton, servo_bb)
    bracket_botton_placed = Pos(0, 0, BRAKET_BOTTOM_Z_OFFSET) * bracket_botton_placed
    
    bracket_inclinded_placed = _center_xy_below(Rot(0, 0, 90) * Rot(bracket_inclined.THETA_DEG, 0, 0) * bracket_inclinded_u_shape, bracket_botton_placed.bounding_box())
    bracket_inclinded_placed = Pos(BRAKET_INCLINED_X_OFFSET, 0, 0) * bracket_inclinded_placed


    femur = Compound(children=[servo, bracket_botton_placed, bracket_inclinded_placed])
    RevoluteJoint("femur_to_coxa_revolute", femur, axis=Axis((0, 0, servo_simplified.HORN_Z_CTR), (0, 1, 0)))
    RigidJoint("femur_to_tibia_fixed", femur, bracket_inclinded_placed.joints["fixed"].location)
    return femur


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    export_step(assembly, "generated/assembly_femur.step")
    export_stl(assembly, "generated/assembly_femur.stl")

    show(assembly, name="assembly_femur", clear=True)


if __name__ == "__main__":
    main()
