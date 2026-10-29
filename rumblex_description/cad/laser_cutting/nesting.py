"""Conservative polygon raster nesting with multiple orders and exact final checks.

Rasterization affects candidate placement, never the exported nominal geometry.
Every occupied polygon cell is covered conservatively, including curved edges.
FFT correlation finds all collision-free translations for each allowed rotation.
"""

from collections import Counter
from dataclasses import dataclass
from math import ceil, floor, sqrt
import random

import numpy as np
from scipy.signal import fftconvolve
from shapely import contains_xy
from shapely.affinity import rotate, translate
from shapely.geometry import box


@dataclass(frozen=True)
class Settings:
    width: float = 300
    height: float = 300
    margin: float = 3
    clearance: float = 1
    kerf: float = 0
    tolerance: float = 0.01
    resolution: float = 1
    rotation_step: float = 30
    trials: int = 3

    def validate(self):
        import math
        for name in ("width", "height", "margin", "clearance", "kerf", "tolerance",
                     "resolution", "rotation_step"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if min(self.width, self.height) <= 2 * self.margin:
            raise ValueError("Margins leave no usable sheet")
        if self.resolution <= 0 or self.tolerance <= 0 or not 0 < self.rotation_step <= 360:
            raise ValueError("Resolution, tolerance and rotation step must be positive (rotation <= 360)")
        if self.trials < 1:
            raise ValueError("At least one nesting trial is required")
        if (self.width * self.height / self.resolution**2) > 4_000_000:
            raise ValueError("Resolution requires more than four million cells per sheet")


@dataclass
class Placement:
    profile: object
    instance: int
    angle: float
    x: float
    y: float
    polygon: object


@dataclass
class Variant:
    angle: float
    x: float
    y: float
    polygon: object
    mask: object


def variants(profile, settings):
    s = settings
    result = []
    for angle in np.arange(0, 360 - 1e-8, s.rotation_step):
        polygon = rotate(profile.polygon, float(angle), origin=(0, 0))
        # Reserve the beam's full swept width as well as the inter-part clearance.
        padding = (s.clearance + s.kerf) / 2 + 2 * s.tolerance
        envelope = polygon.buffer(padding, quad_segs=32, join_style="mitre")
        minx, miny, maxx, maxy = envelope.bounds
        nx, ny = ceil((maxx - minx) / s.resolution), ceil((maxy - miny) / s.resolution)
        if nx * s.resolution > s.width - 2 * s.margin or ny * s.resolution > s.height - 2 * s.margin:
            continue
        polygon = translate(polygon, -minx, -miny)
        envelope = translate(envelope, -minx, -miny)
        # A cell whose centre is within its half-diagonal of the polygon may
        # intersect it. Mitred buffering is conservative at convex corners.
        cover = envelope.buffer(s.resolution / sqrt(2) + 1e-7, join_style="mitre")
        xs = (np.arange(nx) + 0.5) * s.resolution
        ys = (np.arange(ny) + 0.5) * s.resolution
        mask = contains_xy(cover, xs[None, :], ys[:, None])
        result.append(Variant(float(angle), -minx, -miny, polygon, mask))
    if not result:
        raise ValueError(f"{profile.name}: cannot fit a {s.width:g} x {s.height:g} mm sheet "
                         "with these margins, rotations and search resolution")
    return result


def nest(profiles, settings):
    settings.validate()
    s = settings
    options = {p.name: variants(p, s) for p in profiles}
    instances = [(p, i) for p in profiles for i in range(1, p.quantity + 1)]
    orders = [sorted(instances, key=lambda item: item[0].polygon.area, reverse=True),
              sorted(instances, key=lambda item: max(
                  item[0].polygon.bounds[2] - item[0].polygon.bounds[0],
                  item[0].polygon.bounds[3] - item[0].polygon.bounds[1]), reverse=True)]
    rng = random.Random(0)
    best = None
    for trial in range(s.trials):
        if trial < len(orders):
            order = orders[trial]
        else:
            order = sorted(instances, key=lambda item: item[0].polygon.area * rng.uniform(0.7, 1.3),
                           reverse=True)
        sheets, masks = [], []
        for profile, instance in order:
            selected = None
            for sheet_index in range(len(sheets) + 1):
                fresh = sheet_index == len(sheets)
                occupied = np.zeros((floor((s.height - 2*s.margin) / s.resolution),
                                     floor((s.width - 2*s.margin) / s.resolution)), dtype=bool) if fresh else masks[sheet_index]
                for variant in options[profile.name]:
                    h, w = variant.mask.shape
                    collisions = fftconvolve(occupied, variant.mask[::-1, ::-1], mode="valid")
                    rows, cols = np.nonzero(collisions < 0.5)
                    if not len(rows):
                        continue
                    # Bottom-left placement, preferring the lowest resulting top.
                    scores = (rows + h) * (occupied.shape[1] + 1) + cols
                    index = np.argmin(scores)
                    row, col = int(rows[index]), int(cols[index])
                    score = (row + h, col, w)
                    if selected is None or score < selected[0]:
                        selected = (score, sheet_index, row, col, variant)
                if selected is not None:
                    if fresh:
                        sheets.append([])
                        masks.append(occupied)
                    break
            if selected is None:
                raise ValueError(f"Unable to place {profile.name} instance {instance}")
            _, sheet_index, row, col, variant = selected
            dx, dy = s.margin + col * s.resolution, s.margin + row * s.resolution
            sheets[sheet_index].append(Placement(profile, instance, variant.angle,
                variant.x + dx, variant.y + dy, translate(variant.polygon, dx, dy)))
            h, w = variant.mask.shape
            masks[sheet_index][row:row+h, col:col+w] |= variant.mask
        score = (len(sheets), sum(max(p.polygon.bounds[3] for p in sheet) for sheet in sheets))
        if best is None or score < best[0]:
            best = (score, sheets)
    validate(profiles, best[1], s)
    return best[1]


def validate(profiles, sheets, settings):
    s = settings
    required = Counter({(p.name, i): 1 for p in profiles for i in range(1, p.quantity + 1)})
    placed = Counter((p.profile.name, p.instance) for sheet in sheets for p in sheet)
    if placed != required:
        raise ValueError(f"BOM mismatch: missing={required-placed}, excess={placed-required}")
    boundary = box(s.margin, s.margin, s.width-s.margin, s.height-s.margin)
    for sheet in sheets:
        if len({p.profile.thickness for p in sheet}) != 1:
            raise ValueError("Mixed material thickness on a sheet")
        for index, part in enumerate(sheet):
            safe = part.polygon.buffer(s.kerf / 2 + s.tolerance, join_style="mitre")
            if not boundary.covers(safe):
                raise ValueError(f"{part.profile.name}: cutting contour exceeds sheet boundary")
            for other in sheet[:index]:
                if (part.polygon.intersection(other.polygon).area > 1e-8 or
                    part.polygon.distance(other.polygon) + 1e-7 < s.clearance + s.kerf + 2*s.tolerance):
                    raise ValueError(f"Overlap or insufficient clearance: {part.profile.name}, {other.profile.name}")
