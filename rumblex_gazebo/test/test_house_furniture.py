"""Check the furniture's physical geometry and clearance in the house floor plan."""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
MODEL = ET.parse(ROOT / 'rumblex_gazebo/models/house_furniture.sdf').getroot().find('model')
ROOM_BOUNDS = {
    'dining': (-4.31, 0.37, -0.21, 4.51),
    'tv': (-0.09, -4.51, 4.31, -1.11),
    'kitchen': (0.85, 1.33, 4.31, 4.51),
}


def bounds(link, element):
    x, y, z, roll, pitch, yaw = map(float, link.findtext('pose').split())
    assert roll == pitch == 0
    cx, cy, cz, cr, cp, cyaw = map(float, element.findtext('pose').split())
    assert cr == cp == cyaw == 0
    sx, sy, sz = map(float, element.findtext('geometry/box/size').split())
    assert min(sx, sy, sz) > 0
    cosine, sine = math.cos(yaw), math.sin(yaw)
    center = (x + cosine * cx - sine * cy, y + sine * cx + cosine * cy, z + cz)
    half = ((abs(cosine) * sx + abs(sine) * sy) / 2,
            (abs(sine) * sx + abs(cosine) * sy) / 2, sz / 2)
    return tuple(c - h for c, h in zip(center, half)), tuple(c + h for c, h in zip(center, half))


def intersects_xy(lower, upper, rectangle):
    x0, y0, x1, y1 = rectangle
    return lower[0] < x1 and upper[0] > x0 and lower[1] < y1 and upper[1] > y0


def test_requested_furniture_and_static_placement():
    assert MODEL.findtext('static') == 'true'
    assert MODEL.find('pose') is None  # create may override it with the identity pose.
    names = [link.get('name') for link in MODEL.findall('link')]
    assert len(names) == len(set(names))
    assert len([name for name in names if name.startswith('dining_chair_')]) == 6
    assert {'dining_table', 'tv_couch', 'kitchen_sink', 'kitchen_cooker', 'kitchen_fridge'} <= set(names)
    table = MODEL.find("link[@name='dining_table']")
    # Robot-height free space under the tabletop; legs remain real obstacles.
    low, high = bounds(table, table.find("collision[@name='top']"))
    assert low[2] == pytest.approx(0.69)
    assert high[2] == pytest.approx(0.75)
    assert len([c for c in table.findall('collision') if c.get('name').startswith('leg_')]) == 4


@pytest.mark.parametrize('link', MODEL.findall('link'), ids=lambda link: link.get('name'))
def test_visible_parts_match_collisions_and_stay_inside_room(link):
    names = [child.get('name') for child in link if child.get('name') is not None]
    assert len(names) == len(set(names))
    visuals = {v.get('name').removesuffix('_visual'): v for v in link.findall('visual')}
    collisions = {c.get('name'): c for c in link.findall('collision')}
    assert visuals.keys() == collisions.keys()
    x0, y0, x1, y1 = ROOM_BOUNDS[link.get('name').split('_')[0]]
    lowest = math.inf
    for name, collision in collisions.items():
        visual = visuals[name]
        assert collision.findtext('pose') == visual.findtext('pose')
        assert collision.find('geometry')[0].tag == visual.find('geometry')[0].tag == 'box'
        assert collision.findtext('geometry/box/size') == visual.findtext('geometry/box/size')
        lower, upper = bounds(link, collision)
        assert lower[0] >= x0 and lower[1] >= y0
        assert upper[0] <= x1 and upper[1] <= y1
        assert lower[2] >= -1e-9
        lowest = min(lowest, lower[2])
    assert lowest == pytest.approx(0, abs=1e-9)


def test_walls_door_approaches_and_robot_spawn_remain_clear():
    spec = importlib.util.spec_from_file_location(
        'house_map', ROOT / 'rumblex_navigation/scripts/generate_house_map.py')
    house_map = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(house_map)
    # Floor-plan coordinates are measured from the house's southwest corner.
    walls = [(x0 - 4.45, y0 - 4.65, x1 - 4.45, y1 - 4.65)
             for x0, y0, x1, y1 in house_map.OBSTACLES]
    clear_areas = [
        (-1.1, 0.5, -0.21, 4.4),  # dining east aisle and doorway approach
        (-0.09, -3.8, 0.8, -2.3),  # TV entrance from living room
        (1.0, 1.4, 3.4, 2.3),  # kitchen entrance and approach
        (-2.76, -2.63, -1.76, -1.63),  # 1 m square around default robot spawn
    ]
    for link in MODEL.findall('link'):
        for collision in link.findall('collision'):
            lower, upper = bounds(link, collision)
            for rectangle in walls + clear_areas:
                assert not intersects_xy(lower, upper, rectangle), (link.get('name'), rectangle)
