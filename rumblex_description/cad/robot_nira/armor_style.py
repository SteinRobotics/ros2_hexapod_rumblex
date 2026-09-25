"""Nira armor motifs, proportioned from the unchanged outer foot plate.

Coordinates follow a panel's spine (X) and extend across it (Y). The foot
uses a 6 mm sweep over a 7 mm span, 5 mm width and 14 mm repeat pitch.
"""

GILL_SWEEP_RATIO = 6 / 7
GILL_WIDTH_RATIO = 5 / 7
GILL_PITCH_RATIO = 14 / 7


def gill_points(x: float, y: float, span: float, *, side: int = 1):
    """Four corners of a scaled foot gill, mirrored about the panel spine."""
    width = span * GILL_WIDTH_RATIO
    sweep = span * GILL_SWEEP_RATIO
    return [(x, side * y), (x + width, side * y),
            (x + width - sweep, side * (y + span)),
            (x - sweep, side * (y + span))]
