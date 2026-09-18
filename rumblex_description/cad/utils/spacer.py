#!/usr/bin/env python3

import sys
from pathlib import Path

from build123d import BuildPart, Cylinder, Mode, Part, export_step, export_stl

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ocp_utils import show

FOOT_SPACER_OUTER_DIAMETER = 4.0
FOOT_SPACER_OVERALL_LENGTH = 32.0
FOOT_SPACER_STUD_HOLE_DIAMETER = 2.5

def build_model(
    outer_diameter: float = FOOT_SPACER_OUTER_DIAMETER,
    inner_diameter: float = FOOT_SPACER_STUD_HOLE_DIAMETER,
    length: float = FOOT_SPACER_OVERALL_LENGTH,
    build_method=BuildPart,
) -> Part:
    if inner_diameter <= 0:
        raise ValueError("Stud hole diameter must remain positive")

    if outer_diameter <= 0:
        raise ValueError("Outer diameter must remain positive")

    if length <= 0:
        raise ValueError("Length must remain positive")

    if inner_diameter >= outer_diameter:
        raise ValueError("Stud hole diameter must remain smaller than the outer diameter")

    with build_method() as model:
        Cylinder(outer_diameter / 2, length)
        Cylinder(inner_diameter / 2, length, mode=Mode.SUBTRACT)

    return model.part


def main() -> None:
    result = build_model()
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/spacer.step")

    show(result, name="spacer", clear=True)


if __name__ == "__main__":
    main()