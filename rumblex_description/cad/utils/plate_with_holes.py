"""
plate_with_holes.py

Parametric plate with mounting holes, built with build123d.

Design goals:
- All dimensions live in one dataclass (PlateConfig) so the part is
  reusable: change the config, get a different plate.
- Hole placement is a separate, overridable function so the count
  (0-4, via cfg.hole_count) and pattern can change without touching
  build_plate().
"""

import sys
from pathlib import Path
from dataclasses import dataclass
from build123d import *

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ocp_utils import show


@dataclass
class PlateConfig:
    length: float = 80.0
    width: float = 40.0
    thickness: float = 5.0
    corner_radius: float = 4.0
    hole_diameter: float = 5.0
    hole_edge_offset: float = 10.0  # hole center distance from both borders
    hole_count: int = 4  # 0-4 holes, placed at corners
    spacer_height: float = 5.0

    def __post_init__(self) -> None:
        if not 0 <= self.hole_count <= 4:
            raise ValueError("hole_count must be between 0 and 4")
        if self.hole_count and (
            self.hole_edge_offset > self.length / 2
            or self.hole_edge_offset > self.width / 2
        ):
            raise ValueError("hole_edge_offset must be smaller than length/2 and width/2")
        if self.hole_count and self.hole_edge_offset <= self.corner_radius:
            raise ValueError(
                "hole_edge_offset must be larger than corner_radius, "
                "otherwise the hole overlaps the rounded corner"
            )

def two_holes_on_one_side(cfg: PlateConfig) -> list[tuple[float, float]]:
    x = cfg.length / 2 - cfg.hole_edge_offset
    y = cfg.width / 2 - cfg.hole_edge_offset
    return [(x, y), (-x, y)]

def default_hole_positions(cfg: PlateConfig) -> list[tuple[float, float]]:
    n = cfg.hole_count
    if n == 0:
        return []

    x = cfg.length / 2 - cfg.hole_edge_offset
    y = cfg.width / 2 - cfg.hole_edge_offset
    corners = [(x, y), (-x, -y), (-x, y), (x, -y)]
    return corners[:n]


def build_plate(
    cfg: PlateConfig = PlateConfig(),
    hole_positions: list[tuple[float, float]] | None = None,
) -> Part:
    """Build a rounded-corner rectangular plate with holes and spacers.

    Args:
        cfg: plate dimensions.
        hole_positions: (x, y) hole centers relative to plate center.
            Defaults to cfg.hole_count holes via default_hole_positions().
    """
    if hole_positions is None:
        hole_positions = default_hole_positions(cfg)

    with BuildPart() as bp:
        with BuildSketch() as sk:
            Rectangle(cfg.length, cfg.width)
            fillet(sk.vertices(), radius=cfg.corner_radius)
        extrude(amount=cfg.thickness)

        if hole_positions:
            with Locations(*hole_positions):
                Hole(radius=cfg.hole_diameter / 2)

    return bp.part


if __name__ == "__main__":
    result = build_plate()
    Path("generated").mkdir(exist_ok=True)
    export_step(result, "generated/plate_with_holes.step")

    show(result, name="plate_with_holes", clear=True)