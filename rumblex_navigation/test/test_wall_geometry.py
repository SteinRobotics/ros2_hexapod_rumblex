"""Check wall extrusion, door clearance and height-aware lidar intersections."""

import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from house_rooms import ROOMS
from wall_geometry import gazebo_wall_model, ray_distance, rotate, wall_boxes


class WallGeometryTest(unittest.TestCase):
    def test_gazebo_walls_preserve_map_origin_size_and_doorway(self):
        boxes = wall_boxes([100, 0, 100, 100, 0, 100], 3, 2, 0.5, 1.8)
        rotation = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
        model = ET.fromstring(gazebo_wall_model(boxes, (-4.95, -5.15, 0), rotation)).find('model')
        self.assertEqual(model.findtext('static'), 'true')
        self.assertEqual(model.find('link/pose').get('rotation_format'), 'quat_xyzw')
        self.assertEqual(list(map(float, model.findtext('link/pose').split())), [-4.95, -5.15, 0, *rotation])
        collisions = model.findall('link/collision')
        visuals = model.findall('link/visual')
        self.assertEqual(len(collisions), 2)
        self.assertEqual(len(visuals), 2)
        for collision, visual in zip(collisions, visuals):
            self.assertEqual(collision.findtext('pose'), visual.findtext('pose'))
            self.assertEqual(collision.findtext('geometry/box/size'), '0.5 1.0 1.8')
            self.assertEqual(collision.findtext('geometry/box/size'), visual.findtext('geometry/box/size'))
        self.assertEqual([float(c.findtext('pose').split()[0]) for c in collisions], [0.25, 1.25])

    def test_spawn_pose_override_keeps_walls_and_labels_in_map_frame(self):
        rotation = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
        model = ET.fromstring(gazebo_wall_model(
            [((0, 0, 0), (1, 2, 2))], (-4.95, -5.15, 0), rotation, ROOMS)).find('model')
        # Emulate create's identity model-pose override, then compose link poses.
        ET.SubElement(model, 'pose').text = '0 0 0 0 0 0'
        walls = model.find("link[@name='walls']")
        center = tuple(map(float, walls.findtext('visual/pose').split()[:3]))
        center = rotate(center, rotation)
        self.assertAlmostEqual(center[0] - 4.95, -5.95)
        self.assertAlmostEqual(center[1] - 5.15, -4.65)
        labels = model.find("link[@name='room_labels']")
        self.assertIsNone(labels.find('collision'))
        self.assertIsNone(labels.find('pose'))
        for name, x, y in ROOMS:
            ink = [v for v in labels.findall('visual') if v.get('name').startswith(name + '_')]
            self.assertTrue(ink, name)
            bounds = []
            for visual in ink:
                cx, cy, cz = map(float, visual.findtext('pose').split()[:3])
                sx, sy, sz = map(float, visual.findtext('geometry/box/size').split())
                bounds.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
                self.assertGreater(cz - sz / 2, 0)
            self.assertAlmostEqual((min(b[0] for b in bounds) + max(b[1] for b in bounds)) / 2, x)
            self.assertAlmostEqual((min(b[2] for b in bounds) + max(b[3] for b in bounds)) / 2, y)

    def test_merge_and_preserve_door_and_unknown(self):
        data = [100, 0, 100, 100, -1, 100, 100, 100, 100]
        boxes = wall_boxes(data, 3, 3, 0.5, 2.0)
        self.assertEqual(len(boxes), 3)
        self.assertTrue(all(low[2] == 0 and high[2] == 2 for low, high in boxes))
        self.assertEqual(ray_distance((0.75, -1, 1), (0, 1, 0), boxes, 40), 2)
        self.assertEqual(ray_distance((0.25, -1, 1), (0, 1, 0), boxes, 40), 1)

    def test_height_and_pitch(self):
        boxes = [((2, -1, 0), (2.2, 1, 2))]
        self.assertEqual(ray_distance((0, 0, 0.2), (1, 0, 0), boxes, 40), 2)
        self.assertTrue(math.isinf(ray_distance((0, 0, 2.1), (1, 0, 0), boxes, 40)))
        direction = (math.sqrt(0.5), 0, math.sqrt(0.5))
        self.assertTrue(math.isinf(ray_distance((0, 0, 0.2), direction, boxes, 40)))
        self.assertAlmostEqual(ray_distance((0, 0, 0.2), (0.8, 0, 0.6), boxes, 40), 2.5)

    def test_nearest_range_limits_and_inside(self):
        boxes = [((4, -1, 0), (5, 1, 2)), ((2, -1, 0), (3, 1, 2))]
        self.assertEqual(ray_distance((0, 0, 1), (1, 0, 0), boxes, 40), 2)
        self.assertTrue(math.isinf(ray_distance((0, 0, 1), (1, 0, 0), boxes, 1)))
        self.assertTrue(math.isinf(ray_distance((0, 0, 1), (-1, 0, 0), boxes, 40)))
        self.assertEqual(ray_distance((2.5, 0, 1), (1, 0, 0), boxes, 40), 0)
        self.assertTrue(math.isinf(ray_distance((0, 0, 1), (1, 0, 0), [], 40)))

    def test_rotated_grid_and_sensor(self):
        rotation = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
        inverse = (0, 0, -rotation[2], rotation[3])
        direction = rotate((1, 0, 0), rotation)
        self.assertAlmostEqual(direction[0], 0)
        self.assertAlmostEqual(direction[1], 1)
        local = rotate(direction, inverse)
        self.assertAlmostEqual(local[0], 1)
        self.assertAlmostEqual(local[1], 0)


if __name__ == '__main__':
    unittest.main()
