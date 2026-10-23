# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import *
from utils.ocp_utils import show

from utils.plate_with_holes import PlateConfig, default_hole_positions
import utils.spacer as spacer
from utils.colors import *

GRID_PITCH = 2.54
PCB_THICKNESS = 1.6
PIN_HOLE_DIAMETER = 1.0
# Simplified 5264-3P clearance envelope, using the requested 2.54 mm pins.
PLUG_LENGTH = 9.9
PLUG_WIDTH = 4.9
PLUG_HEIGHT = 8.4
PIN_SIZE = 0.64
PLUG_COLUMNS = 5
PLUG_ROWS = 2
# Smallest whole-grid steps that accommodate the rotated 4.9 x 9.9 mm bodies.
PLUG_COLUMN_PITCH = 2 * GRID_PITCH
PLUG_ROW_PITCH = 4 * GRID_PITCH
PLUG_POSITIONS = tuple(
    ((column - (PLUG_COLUMNS - 1) / 2) * PLUG_COLUMN_PITCH,
     (row - (PLUG_ROWS - 1) / 2) * PLUG_ROW_PITCH)
    for row in range(PLUG_ROWS)
    for column in range(PLUG_COLUMNS)
)

cfg = PlateConfig(
    length = (PLUG_COLUMNS + 2) * PLUG_COLUMN_PITCH,
    width = 10 * GRID_PITCH,
    thickness = PLUG_HEIGHT + PCB_THICKNESS,
    corner_radius = 1.0,
    hole_diameter = 2.0,
    hole_edge_offset = GRID_PITCH,
    hole_count = 4,
    spacer_height = 4.0,
)


def pin_positions() -> list[tuple[float, float]]:
    """The 30 solder holes lie on the same grid as the four spacer holes."""
    return [(x, y + pin * GRID_PITCH)
            for x, y in PLUG_POSITIONS for pin in (-1, 0, 1)]


def build_surface() -> Sketch:
    hole_positions = default_hole_positions(cfg)
    with BuildSketch() as sk:
        Rectangle(cfg.length, cfg.width)
        fillet(sk.vertices(), radius=cfg.corner_radius)
        for x, y in hole_positions:
            with Locations((x, y)):
                Circle(cfg.hole_diameter / 2, mode=Mode.SUBTRACT)
        with Locations(*pin_positions()):
            Circle(PIN_HOLE_DIAMETER / 2, mode=Mode.SUBTRACT)
    return sk.sketch


def build_plug() -> Compound:
    """Three sockets and contacts; local Z=0 is the mating face.

    Like the other board modules, the PCB back is at cfg.thickness and
    the spacers extend in +Z. The torso assembly flips the component side up.
    """
    with BuildPart() as housing:
        Box(PLUG_LENGTH, PLUG_WIDTH, PLUG_HEIGHT,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations(*[(pin * GRID_PITCH, 0, 0) for pin in (-1, 0, 1)]):
            Box(1.5, 1.5, PLUG_HEIGHT, align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT)
    housing.part.color = COLOR_CREAMY_WHITE
    housing.part.label = "5264-3P_housing"
    contacts = []
    for pin in (-1, 0, 1):
        contact = Pos(pin * GRID_PITCH, 0, 2.0) * Box(
            PIN_SIZE, PIN_SIZE, cfg.thickness - 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        contact.color = COLOR_BRASS
        contact.label = f"contact_{pin + 2}"
        contacts.append(contact)
    return Compound(label="5264-3P", children=[housing.part, *contacts])


def build_model() -> Compound:
    """One PCB carrying ten connectors in two tightly packed rows of five."""
    with BuildPart() as pcb:
        insert(Pos(0, 0, PLUG_HEIGHT) * build_surface())
        extrude(amount=PCB_THICKNESS)
    pcb.part.color = COLOR_DARK_GREEN
    pcb.part.label = "pcb"
    template = Rot(Z=90) * build_plug()
    plugs = []
    for index, (x, y) in enumerate(PLUG_POSITIONS, 1):
        plug = Pos(x, y) * template
        plug.label = f"servo_plug_{index}"
        plugs.append(plug)
    return Compound(label="board_servo_plug", children=[pcb.part, *plugs])


def build_board_with_spacers() -> Compound:
    hole_positions = default_hole_positions(cfg)
    board = build_model()
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
    for s in spacers: s.color = COLOR_LIGHT_GRAY
    return Compound(label="board_servo_plug_with_spacers", children=[board, *spacers])


if __name__ == "__main__":
    result = build_board_with_spacers()
    show(result, name="board_servo_plug", clear=True)
