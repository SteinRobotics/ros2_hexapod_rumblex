"""Nira's forward sensor deck and swept rear canopy, in millimetres.

+X is forward. The 270-degree clear sector is -135..+135 degrees from +X.
The canopy edge x + abs(y) = 44 leaves 16 mm behind its boundary at x=60.
"""

from build123d import BuildSketch, Polygon, Sketch

LIDAR_X = 60.0
LIDAR_Y = 0.0
LOW_RAIL_HEIGHT = 8.0
CANOPY_TIP_X = 44.0


def canopy_surface() -> Sketch:
    """Faceted arrowhead tail; all tall body structure stays behind this edge."""
    with BuildSketch() as sketch:
        Polygon(
            (-110, -45), (-90, -65), (-21, -65), (CANOPY_TIP_X, 0),
            (-21, 65), (-90, 65), (-110, 45), align=None,
        )
    return sketch.sketch
