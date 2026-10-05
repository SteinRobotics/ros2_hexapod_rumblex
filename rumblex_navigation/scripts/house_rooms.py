"""Shared house room positions and floor lettering for Gazebo."""

# Map-frame coordinates, measured from the current house floor plan.
ROOMS = (
    ('Dining', -2.26, 2.44),
    ('Kitchen', 2.58, 2.92),
    ('WC', 0.32, 3.49),
    ('Hallway', 0.35, 1.75),
    ('Living', -2.26, -2.13),
    ('TV', 2.11, -2.81),
)


# Five-column, seven-row capitals. Floor geometry needs no GUI marker bridge,
# external font, texture paths, or collision shapes.
GLYPHS = {
    'A': ('01110', '10001', '10001', '11111', '10001', '10001', '10001'),
    'C': ('01111', '10000', '10000', '10000', '10000', '10000', '01111'),
    'D': ('11110', '10001', '10001', '10001', '10001', '10001', '11110'),
    'E': ('11111', '10000', '10000', '11110', '10000', '10000', '11111'),
    'G': ('01111', '10000', '10000', '10111', '10001', '10001', '01111'),
    'H': ('10001', '10001', '10001', '11111', '10001', '10001', '10001'),
    'I': ('11111', '00100', '00100', '00100', '00100', '00100', '11111'),
    'K': ('10001', '10010', '10100', '11000', '10100', '10010', '10001'),
    'L': ('10000', '10000', '10000', '10000', '10000', '10000', '11111'),
    'N': ('10001', '11001', '11001', '10101', '10011', '10011', '10001'),
    'T': ('11111', '00100', '00100', '00100', '00100', '00100', '00100'),
    'V': ('10001', '10001', '10001', '10001', '10001', '01010', '00100'),
    'W': ('10001', '10001', '10001', '10101', '10101', '10101', '01010'),
    'Y': ('10001', '10001', '01010', '00100', '00100', '00100', '00100'),
}


def floor_lettering(name, x, y, height=0.24):
    """Yield horizontal ink runs as (center, size), centered on a room position."""
    pixel = height / 7
    width = (len(name) * 6 - 1) * pixel
    for index, character in enumerate(name.upper()):
        for row, bits in enumerate(GLYPHS[character]):
            col = 0
            while col < len(bits):
                if bits[col] == '0':
                    col += 1
                    continue
                start = col
                while col < len(bits) and bits[col] == '1':
                    col += 1
                yield ((x - width / 2 + (index * 6 + (start + col) / 2) * pixel,
                        y + height / 2 - (row + 0.5) * pixel, 0.025),
                       ((col - start) * pixel, pixel, 0.002))
