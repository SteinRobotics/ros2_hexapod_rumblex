#!/usr/bin/env python3
"""Closed support ring mounted against the underside of body layer 2."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import (
    BuildPart,
    BuildSketch,
    ExportDXF,
    Mode,
    Part,
    Sketch,
    insert,
    export_step,
    extrude,
    offset,
)

from robot_nira import EXPORT_DIR, dxf_directory
import robot_nira.torso_layer_2_cover as torso_layer_2_cover
from utils.ocp_utils import show


THICKNESS = 1.0
FINGER_WIDTH = 6.0


def build_surface() -> Sketch:
    """Follow the deck opening with a continuous rim and a cover support ledge."""
    opening_outline = torso_layer_2_cover.build_opening_outline(clearance=0.0)
    outer = offset(opening_outline, amount=FINGER_WIDTH / 2)
    inner = offset(opening_outline, amount=-FINGER_WIDTH / 2)
    cable_cutouts = torso_layer_2_cover.build_cable_cutouts(opening_outline, stop_at_edge=True)
    # At the angled rear edges, rectangular cutters can cross the outer rim.
    # Limit reliefs to the opening so the ring remains continuous everywhere.
    cable_cutouts = cable_cutouts & opening_outline
    with BuildSketch() as sketch:
        insert(outer)
        insert(inner, mode=Mode.SUBTRACT)
        insert(cable_cutouts, mode=Mode.SUBTRACT)
    return sketch.sketch


def build_model(surface: Sketch) -> Part:
    with BuildPart() as model:
        insert(surface)
        extrude(amount=THICKNESS)
    return model.part


def main() -> None:
    surface = build_surface()
    result = build_model(surface)

    (EXPORT_DIR / "step").mkdir(parents=True, exist_ok=True)
    dxf_dir = dxf_directory(THICKNESS)
    export_step(result, str(EXPORT_DIR / "step/torso_layer_2_addon.step"))

    dxf_export = ExportDXF()
    dxf_export.add_shape(surface)
    dxf_export.write(str(dxf_dir / "torso_layer_2_addon.dxf"))

    show(result, name="torso_layer_2_addon", clear=True)


if __name__ == "__main__":
    main()
