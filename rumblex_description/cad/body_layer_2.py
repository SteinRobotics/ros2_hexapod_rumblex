#!/usr/bin/env python3

from pathlib import Path
from build123d import *
from utils.ocp_utils import show

import body_common

def build_surface() -> Sketch:
    with BuildSketch() as sketch:
        add(body_common.build_surface())

        for points in body_common.SERVO_FRONT_CUTOUTS.values():
            Polygon(*points, mode=Mode.SUBTRACT)

        for x, y, radius in body_common.LIST_SERVO_BRACKET_HOLES:
            with Locations((x, y)):
                Circle(radius, mode=Mode.SUBTRACT)

    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        add(surface)
        extrude(amount=body_common.THICKNESS)

    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/body_layer_2.step")

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write("generated/body_layer_2.dxf")

    show(result, name="body_layer_2", clear=True)

        
if __name__ == "__main__":
    main()
