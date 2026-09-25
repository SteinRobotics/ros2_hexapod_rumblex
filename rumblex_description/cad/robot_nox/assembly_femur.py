#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from robot_nox import EXPORT_DIR

from build123d import Compound, Pos, Rot, export_step, export_stl, import_step, RevoluteJoint, RigidJoint


from utils.colors import COLOR_DARK_GRAY
import common.servo_simplified as servo_simplified
import common.bracket_inclined as bracket_inclined
from utils.ocp_utils import show

# Retained fit offset for the vendor STEP; verify against the physical bracket.
BRACKET_BOTTOM_INSERTION = 5.65

# Turn the femur over about its length, keeping both horns seated in the coxa.
FEMUR_ROLL = 180.0  # degrees about local Z

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

    bracket_bottom = import_step(str(Path(__file__).resolve().parents[1] / "imported" / "HX-35HM Botton Bracket.STEP"))
    bracket_bottom.color = COLOR_DARK_GRAY

    inclined_bracket = bracket_inclined.build_bracket()
    inclined_bracket.color = COLOR_DARK_GRAY

    servo_bb = servo.bounding_box()
    bracket_bottom_placed = _center_xy_below(Rot(180, 0, 90) * bracket_bottom, servo_bb)
    bracket_bottom_placed = Pos(0, 0, BRACKET_BOTTOM_INSERTION) * bracket_bottom_placed
    
    inclined_placed = Rot(0, 0, 90) * Rot(bracket_inclined.THETA_DEG, 0, 0) * inclined_bracket
    mount = inclined_placed.joints["plate_mount"].location.position
    bottom_box = bracket_bottom_placed.bounding_box()
    inclined_placed = Pos(
        bottom_box.center().X - mount.X,
        bottom_box.center().Y - mount.Y,
        bottom_box.min.Z - mount.Z,
    ) * inclined_placed


    back_horn_face_y = (
        servo_simplified.BODY_Y
        + servo_simplified.HORN_DISTANCE_TO_BODY
        + servo_simplified.HORN_BACK_Y
    )
    horn_mid_y = (servo_simplified.HORN_FRONT_FACE_Y + back_horn_face_y) / 2
    roll = Pos(0, horn_mid_y, 0) * Rot(0, 0, FEMUR_ROLL) * Pos(0, -horn_mid_y, 0)
    servo = roll * servo
    bracket_bottom_placed = roll * bracket_bottom_placed
    inclined_placed = roll * inclined_placed

    femur = Compound(children=[servo, bracket_bottom_placed, inclined_placed])
    # The rear horn now occupies the original front mating face. Keep the
    # incoming interface fixed; the tibia interface follows the rolled bracket.
    RevoluteJoint("femur_to_coxa_revolute", femur, axis=servo_simplified.HORN_AXIS)
    RigidJoint("femur_to_tibia_fixed", femur, inclined_placed.joints["fixed"].location)
    return femur


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "stl").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / "step/assembly_femur.step"))
    export_stl(assembly, str(EXPORT_DIR / "stl/assembly_femur.stl"))

    show(assembly, name="assembly_femur", clear=True)


if __name__ == "__main__":
    main()
