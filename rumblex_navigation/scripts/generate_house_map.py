"""Rasterize the ground-floor plan in metres; coordinates start at lower left.

Walls/openings are approximated from the supplied scan, using the printed
8.90 x 9.30 m footprint. Furniture and door swing arcs are not obstacles.
"""

from pathlib import Path


RESOLUTION = 0.02
WIDTH, HEIGHT = 495, 515
MARGIN = 0.50
# Rectangles: xmin, ymin, xmax, ymax, measured from the outer lower-left corner.
OBSTACLES = [
    (0.00, 0.00, 8.90, 0.14),  # exterior
    (0.00, 9.16, 8.90, 9.30),
    (0.00, 0.14, 0.14, 9.16),
    (8.76, 0.14, 8.90, 9.16),
    (4.24, 7.00, 4.36, 9.16),  # upper room / WC; doorway below
    (4.24, 4.90, 4.36, 5.98),
    (0.14, 4.90, 4.36, 5.02),  # upper / lower left rooms
    (5.18, 7.12, 5.30, 9.16),  # WC / kitchen
    (5.18, 7.12, 5.80, 7.58),  # chimney beside WC
    (5.68, 7.00, 5.80, 7.58),  # kitchen doorway north jamb
    (5.50, 5.86, 8.76, 5.98),  # kitchen / stair enclosure
    (5.38, 3.66, 8.76, 5.86),  # stairs including landing: non-traversable
    (4.24, 3.54, 8.76, 3.66),  # lower right room / stairs
    (4.24, 2.44, 4.36, 3.66),  # lower rooms divider, wide opening below
    (4.24, 0.14, 4.36, 0.74),
    (4.24, 3.10, 4.90, 3.54),  # chimney in lower room
]


def generate():
    pixels = bytearray([205]) * (WIDTH * HEIGHT)
    for row in range(HEIGHT):
        y = (HEIGHT - row - 0.5) * RESOLUTION - MARGIN
        for col in range(WIDTH):
            x = (col + 0.5) * RESOLUTION - MARGIN
            if 0 <= x < 8.90 and 0 <= y < 9.30:
                occupied = any(x0 <= x < x1 and y0 <= y < y1
                               for x0, y0, x1, y1 in OBSTACLES)
                pixels[row * WIDTH + col] = 0 if occupied else 254
    header = f'P5\n# Ground floor: walls and blocked stairs; 0.02 m/cell\n{WIDTH} {HEIGHT}\n255\n'
    return header.encode() + pixels


if __name__ == '__main__':
    target = Path(__file__).resolve().parents[1] / 'maps' / 'house_contour.pgm'
    target.write_bytes(generate())
