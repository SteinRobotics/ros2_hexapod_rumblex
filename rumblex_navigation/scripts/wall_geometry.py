"""Extrude occupied grid cells and intersect a lidar ray with the resulting boxes."""

import math


def wall_boxes(data, width, height, resolution, wall_height):
    """Merge equal occupied row runs into boxes in the grid origin frame."""
    boxes = []
    active = {}
    for row in range(height + 1):
        runs = set()
        col = 0
        while row < height and col < width:
            if data[row * width + col] < 65:
                col += 1
                continue
            start = col
            while col < width and data[row * width + col] >= 65:
                col += 1
            runs.add((start, col))
        for run in list(active):
            if run not in runs:
                start_row = active.pop(run)
                boxes.append(((run[0] * resolution, start_row * resolution, 0.0),
                              (run[1] * resolution, row * resolution, wall_height)))
        for run in sorted(runs):
            active.setdefault(run, row)
    return boxes


def rotate(vector, quaternion):
    """Rotate a vector by a unit quaternion (x, y, z, w)."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def ray_distance(origin, direction, boxes, max_range):
    """Nearest box along a unit ray; infinity when no surface is in range."""
    nearest = math.inf
    for lower, upper in boxes:
        entry, leave = 0.0, max_range
        for position, delta, low, high in zip(origin, direction, lower, upper):
            if abs(delta) < 1e-12:
                if position < low or position > high:
                    break
            else:
                first, last = sorted(((low - position) / delta, (high - position) / delta))
                entry, leave = max(entry, first), min(leave, last)
            if entry > leave:
                break
        else:
            nearest = min(nearest, entry)
    return nearest
