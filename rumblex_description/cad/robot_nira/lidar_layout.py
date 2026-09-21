"""Nira's forward sensor deck and swept rear canopy, in millimetres.

+X is forward. The 270-degree clear sector is -135..+135 degrees from +X.
The canopy tip ends above the scanner axis. Side shoulders stay behind the
scan sector; the canopy itself sits above the complete scanner envelope.
"""

from build123d import BuildSketch, Polygon, Sketch

LIDAR_X = 80.0
LIDAR_Y = 0.0
LOW_RAIL_HEIGHT = 8.0
CANOPY_TIP_X = LIDAR_X
CANOPY_HALF_WIDTH = 65.0
SHOULDER_TOP_X = CANOPY_TIP_X - CANOPY_HALF_WIDTH - 10.0


def canopy_surface() -> Sketch:
    """Faceted arrowhead roof with its nose over the lidar axis."""
    with BuildSketch() as sketch:
        Polygon(
            (-110, -45), (-90, -CANOPY_HALF_WIDTH),
            (CANOPY_TIP_X - CANOPY_HALF_WIDTH, -CANOPY_HALF_WIDTH),
            (CANOPY_TIP_X, LIDAR_Y),
            (CANOPY_TIP_X - CANOPY_HALF_WIDTH, CANOPY_HALF_WIDTH),
            (-90, CANOPY_HALF_WIDTH), (-110, 45), align=None,
        )
    return sketch.sketch
