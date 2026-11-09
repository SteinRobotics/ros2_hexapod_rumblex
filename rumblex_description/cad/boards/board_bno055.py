# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path
from build123d import *
from utils.ocp_utils import show

from utils.plate_with_holes import build_plate, PlateConfig, two_holes_on_one_side
import utils.spacer as spacer
from utils.colors import *

CHIP_LENGTH = 5.4
CHIP_WIDTH = 4.0
CHIP_HEIGHT = 1.0
# Chip lower-left corner measured from the PCB lower-left corner.
CHIP_OFFSET_X = 10.0
CHIP_OFFSET_Y = 8.0

CAPACITOR_DIAMETER = 2.0
CAPACITOR_HEIGHT = 6.5
CAPACITOR_OFFSET_FROM_LEFT_HOLE = 12.0  # center-to-center, toward the right hole

cfg = PlateConfig(
    length = 21.0,  # along the mounting-hole row
    width = 18.5,
    thickness = 1.5,
    corner_radius = 1.0,
    hole_diameter = 2.5,
    hole_edge_offset = 2.5,  # 21 - 2 * 2.5 = 16 mm between hole centers
    hole_count = 2,
    spacer_height = 2.0,
)

def build_surface() -> Sketch:
    hole_positions = two_holes_on_one_side(cfg)
    with BuildSketch() as sk:
        Rectangle(cfg.length, cfg.width)
        fillet(sk.vertices(), radius=cfg.corner_radius)
        for x, y in hole_positions:
            with Locations((x, y)):
                Circle(cfg.hole_diameter / 2, mode=Mode.SUBTRACT)
    return sk.sketch


def build_chip() -> Part:
    chip = Pos(-cfg.length / 2 + CHIP_OFFSET_X,
               -cfg.width / 2 + CHIP_OFFSET_Y,
               cfg.thickness) * Box(
                   CHIP_LENGTH, CHIP_WIDTH, CHIP_HEIGHT,
                   align=(Align.MIN, Align.MIN, Align.MIN))
    chip.color = COLOR_DARK_GRAY
    chip.label = "bno055_chip"
    return chip


def build_capacitor() -> Part:
    left_x, hole_y = min(two_holes_on_one_side(cfg), key=lambda point: point[0])
    capacitor = Pos(left_x + CAPACITOR_OFFSET_FROM_LEFT_HOLE,
                    hole_y, cfg.thickness) * Cylinder(
                        CAPACITOR_DIAMETER / 2, CAPACITOR_HEIGHT,
                        align=(Align.CENTER, Align.CENTER, Align.MIN))
    capacitor.color = COLOR_DARK_GRAY
    capacitor.label = "bno055_capacitor"
    return capacitor


def build_board_with_spacers() -> Compound:
    hole_positions = two_holes_on_one_side(cfg)
    board = build_plate(cfg=cfg, hole_positions=hole_positions)
    spacer_z = -cfg.spacer_height / 2  # top of each spacer meets the PCB bottom at Z=0
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
    populated_board = Compound(
        label="bno055_board", children=[board, build_chip(), build_capacitor()])
    return Compound(children=[populated_board, *spacers])


if __name__ == "__main__":
    surface = build_surface()
    result = build_board_with_spacers()

    # Path("generated").mkdir(exist_ok=True)
    # export_step(result, "generated/step/board_bno055.step")

    # dxf_export = ExportDXF()
    # dxf_export.add_shape(surface)
    # dxf_export.write("generated/dxf/board_bno055.dxf")

    show(result, name="board_bno055", clear=True)
    