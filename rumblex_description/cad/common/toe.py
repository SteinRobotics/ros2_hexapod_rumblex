#!/usr/bin/env python3

# the toe is a truncated cone

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

from build123d import BuildPart, Cone, Cylinder, Mode, Part

from common.export_utils import export_common_part
from utils.ocp_utils import show

LITTLE_TOE_DIAMETER = 10.0
BIG_TOE_DIAMETER = 14.0
LITTLE_TOE_LENGTH = 5.0
STUD_HOLE_DIAMETER = 2.5


def build_model(
    big_diameter: float = BIG_TOE_DIAMETER,
    little_diameter: float = LITTLE_TOE_DIAMETER,
    inner_diameter: float = STUD_HOLE_DIAMETER,
    length: float = LITTLE_TOE_LENGTH,
    build_method=BuildPart,
) -> Part:
    if inner_diameter <= 0:
        raise ValueError("Stud hole diameter must remain positive")

    if little_diameter <= 0 or big_diameter <= 0:
        raise ValueError("Toe diameters must remain positive")

    if length <= 0:
        raise ValueError("Length must remain positive")

    if inner_diameter >= little_diameter or inner_diameter >= big_diameter:
        raise ValueError("Stud hole diameter must remain smaller than both toe diameters")

    with build_method() as model:
        Cone(bottom_radius=big_diameter / 2, top_radius=little_diameter / 2, height=length)
        Cylinder(inner_diameter / 2, length, mode=Mode.SUBTRACT)

    return model.part


def main() -> None:
    result = build_model()
    export_common_part(result, "toe")

    show(result, name="toe", clear=True)


if __name__ == "__main__":
    main()
