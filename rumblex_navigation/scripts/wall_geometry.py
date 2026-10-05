"""Extrude occupied grid cells and intersect a lidar ray with the resulting boxes."""

import math
import xml.etree.ElementTree as ET

from house_rooms import floor_lettering


def gazebo_wall_model(boxes, position, quaternion, rooms=()):
    """Build one static SDF model with matching visible and collidable map walls."""
    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name='map_walls')
    ET.SubElement(model, 'static').text = 'true'
    # ros_gz_sim create overrides the model pose, including when no pose flags
    # are given. Keep the grid-to-map transform on the link instead.
    link = ET.SubElement(model, 'link', name='walls')
    ET.SubElement(link, 'pose', rotation_format='quat_xyzw').text = ' '.join(
        str(v) for v in (*position, *quaternion))
    for index, (lower, upper) in enumerate(boxes):
        center = [(a + b) / 2 for a, b in zip(lower, upper)]
        size = ' '.join(str(b - a) for a, b in zip(lower, upper))
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name=f'wall_{index}_{kind}')
            ET.SubElement(element, 'pose').text = ' '.join(str(v) for v in (*center, 0, 0, 0))
            geometry = ET.SubElement(element, 'geometry')
            ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = size
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                ET.SubElement(material, 'ambient').text = '0.65 0.7 0.8 1'
                ET.SubElement(material, 'diffuse').text = '0.65 0.7 0.8 1'
    if rooms:
        labels = ET.SubElement(model, 'link', name='room_labels')
        # Room positions already use map coordinates, independent of grid origin.
        for name, x, y in rooms:
            for index, (center, size) in enumerate(floor_lettering(name, x, y)):
                visual = ET.SubElement(labels, 'visual', name=f'{name}_{index}')
                ET.SubElement(visual, 'cast_shadows').text = 'false'
                ET.SubElement(visual, 'pose').text = ' '.join(str(v) for v in (*center, 0, 0, 0))
                geometry = ET.SubElement(visual, 'geometry')
                ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))
                material = ET.SubElement(visual, 'material')
                ET.SubElement(material, 'ambient').text = '0.08 0.08 0.08 1'
                ET.SubElement(material, 'diffuse').text = '0.08 0.08 0.08 1'
    return ET.tostring(sdf, encoding='unicode')


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
