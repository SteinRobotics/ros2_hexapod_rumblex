from pathlib import Path
from build123d import *
from utils.ocp_utils import show

from utils.plate_with_holes import build_plate, PlateConfig, default_hole_positions
import utils.spacer as spacer
from utils.colors import *

cfg = PlateConfig(
    length = 40.0,
    width = 20.0,
    thickness = 10.0, 
    corner_radius = 1.0,
    hole_diameter = 2.2,
    hole_edge_offset = 2.0,
    hole_count = 4,
    spacer_height = 4.0,
)

def build_surface() -> Sketch:
    hole_positions = default_hole_positions(cfg)
    with BuildSketch() as sk:
        Rectangle(cfg.length, cfg.width)
        fillet(sk.vertices(), radius=cfg.corner_radius)
        for x, y in hole_positions:
            with Locations((x, y)):
                Circle(cfg.hole_diameter / 2, mode=Mode.SUBTRACT)
    return sk.sketch


def build_board_with_spacers() -> Compound:
    hole_positions = default_hole_positions(cfg)
    board = build_plate(cfg=cfg)
    spacer_z = cfg.thickness + cfg.spacer_height / 2
    spacers = [
        Pos(x, y, spacer_z)
        * spacer.build_model(
            inner_diameter=cfg.hole_diameter,
            outer_diameter=cfg.hole_diameter + 2,
            length=cfg.spacer_height,
        )
        for x, y in hole_positions
    ]
    board.color = COLOR_DARK_GREEN
    for s in spacers: s.color = COLOR_LIGHT_GRAY
    return Compound(children=[board, *spacers])


if __name__ == "__main__":
    surface = build_surface()
    result = build_board_with_spacers()

    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/board_relay.step")

    show(result, name="board_relay", clear=True)
    