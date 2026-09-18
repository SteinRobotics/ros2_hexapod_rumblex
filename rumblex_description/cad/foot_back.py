#!/usr/bin/env python3

from pathlib import Path

from build123d import (
    BuildPart,
    BuildSketch,
    Circle,
    ExportDXF,
    Locations,
    Mode,
    Part,
    Polygon,
    Rectangle,
    Sketch,
    add,
    export_step,
    extrude,
)

from utils.ocp_utils import show

from servo_cutouts import (SERVO_BACK_CUTOUT, 
                           SERVO_BRACKET_HOLES)

from foot_common import (THICKNESS, 
                         FOOT_OUTLINE, 
                         FOOT_MOUNT_HOLES, 
                         TIP_SLOTS)

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        Polygon(*FOOT_OUTLINE)
        Polygon(*SERVO_BACK_CUTOUT, mode=Mode.SUBTRACT)

        for x, y, radius in SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

        for hole in FOOT_MOUNT_HOLES:
            with Locations((hole["x"], hole["y"])):
                Circle(hole["radius"], mode=Mode.SUBTRACT)

        for x, y, width, height in TIP_SLOTS:
            with Locations((x, y)):
                Rectangle(width, height, mode=Mode.SUBTRACT)

    return sketch.sketch

def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=THICKNESS)

    return model.part

def main() -> None:
    surface = build_surface()
    result = build_model(surface)
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/foot_back.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/foot_back.dxf")

    show(result, name="foot_back", clear=True)


if __name__ == "__main__":
    main()
