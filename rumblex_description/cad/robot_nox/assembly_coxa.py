#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nox import EXPORT_DIR

from build123d import Compound, Pos, Rot, export_step, export_stl, RigidJoint


from utils.colors import COLOR_DARK_GRAY
import common.bracket_inclined as bracket_inclined
import common.bracket_u_shape as bracket_u_shape
from utils.ocp_utils import show

def _center_xy_below(part, reference_bb):
    """Centre the bracket's top mounting face under the reference plate."""
    mount = part.joints["plate_mount"].location.position
    return Pos(
        reference_bb.center().X - mount.X,
        reference_bb.center().Y - mount.Y,
        reference_bb.min.Z - mount.Z,
    ) * part


def build_assembly() -> Compound:
    bracket_inclined_part = bracket_inclined.build_bracket()
    bracket_inclined_part.color = COLOR_DARK_GRAY
    
    bracket_straight = bracket_u_shape.build_bracket()
    bracket_straight.color = COLOR_DARK_GRAY

    bracket_straight_placed = Rot(180, 0, 90) * bracket_straight
    
    bracket_inclined_placed = _center_xy_below(
        Rot(bracket_inclined.THETA_DEG, 0, 0) * bracket_inclined_part,
        bracket_straight_placed.bounding_box(),
    )


    coxa = Compound(children=[bracket_straight_placed, bracket_inclined_placed])
    RigidJoint("body_to_coxa_fixed", coxa, bracket_straight_placed.joints["fixed"].location)
    RigidJoint("coxa_to_femur_fixed", coxa, bracket_inclined_placed.joints["fixed"].location)
    return coxa


def main() -> None:
    assembly = build_assembly()
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / "assembly_coxa.step"))
    export_stl(assembly, str(EXPORT_DIR / "assembly_coxa.stl"))
    show(assembly, name="assembly_coxa", clear=True)


if __name__ == "__main__":
    main()
