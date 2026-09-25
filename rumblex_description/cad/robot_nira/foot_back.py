#!/usr/bin/env python3

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_nira import EXPORT_DIR

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Polygon,
    Sketch,
    add,
    export_step,
    extrude,
)

from utils.ocp_utils import show

from common.servo_simplified import SERVO_BACK_CUTOUT, SERVO_BRACKET_HOLES

from robot_nira.foot_common import THICKNESS, FOOT_MOUNT_HOLES
from robot_nira.foot_common import build_surface as build_common_surface


def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(build_common_surface())
        Polygon(*SERVO_BACK_CUTOUT, align=None, mode=Mode.SUBTRACT)

        for x, y, radius in SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

    return sketch.sketch

def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)

    return model.part

def main() -> None:
    surface = build_surface()
    result = build_model(surface)
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    (EXPORT_DIR / "dxf").mkdir(parents=True, exist_ok=True)
    export_step(result, str(EXPORT_DIR / "step/foot_back.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(EXPORT_DIR / "dxf/foot_back.dxf"))

    show(result, name="foot_back", clear=True)


if __name__ == "__main__":
    main()
