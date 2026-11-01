"""Mesh containment and segment alignment without loading a CAD kernel."""
import math
from pathlib import Path
import struct
import unittest

from test_nira_urdf import expand as expand_nira
from test_nox_urdf import expand as expand_nox

ROOT = Path(__file__).resolve().parents[1]


def rotate(point, rpy):
    x, y, z = point
    r, p, yaw = rpy
    y, z = math.cos(r) * y - math.sin(r) * z, math.sin(r) * y + math.cos(r) * z
    x, z = math.cos(p) * x + math.sin(p) * z, -math.sin(p) * x + math.cos(p) * z
    return (math.cos(yaw) * x - math.sin(yaw) * y,
            math.sin(yaw) * x + math.cos(yaw) * y, z)


def values(element, attr, default='0 0 0'):
    return tuple(map(float, element.get(attr, default).split()))


class LegCollisionTests(unittest.TestCase):
    def test_femur_boxes_follow_joint_centres(self):
        for expand in (expand_nox, expand_nira):
            model = expand(mesh=True)
            for side in ('left', 'right'):
                for position in ('front', 'mid', 'back'):
                    prefix = f'{side}_{position}'
                    joint = model.find(f"joint[@name='{prefix}_tibia_joint']/origin")
                    box = model.find(f"link[@name='{prefix}_femur_link']/collision/origin")
                    axis = rotate((1, 0, 0), values(box, 'rpy'))
                    dx, dy, _ = values(joint, 'xyz')
                    self.assertAlmostEqual(axis[0] * dy - axis[1] * dx, 0, places=10)

    def test_femur_boxes_enclose_every_mesh_vertex(self):
        for robot, expand in (('nox', expand_nox), ('nira', expand_nira)):
            with self.subTest(robot=robot):
                link = expand(mesh=True).find("link[@name='right_mid_femur_link']")
                visual = link.find('visual/origin')
                box = link.find('collision/origin')
                center = values(box, 'xyz')
                rotation = values(box, 'rpy')
                axes = [rotate(unit, rotation) for unit in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
                half = [d / 2 for d in values(link.find('collision/geometry/box'), 'size')]
                data = (ROOT / f'meshes/{robot}/assembly_femur.stl').read_bytes()
                count = struct.unpack_from('<I', data, 80)[0]
                self.assertEqual(len(data), 84 + count * 50)
                maxima = [0., 0., 0.]
                for offset in range(84, len(data), 50):
                    for vertex in range(3):
                        point = struct.unpack_from('<3f', data, offset + 12 + vertex * 12)
                        point = rotate([v * .001 for v in point], values(visual, 'rpy'))
                        delta = [v + t - c for v, t, c in zip(point, values(visual, 'xyz'), center)]
                        for i, axis in enumerate(axes):
                            maxima[i] = max(maxima[i], abs(sum(a * d for a, d in zip(axis, delta))))
                for extent, bound in zip(maxima, half):
                    self.assertLessEqual(extent, bound)
                    self.assertLess(bound - extent, .00011)

    def test_nira_primitive_and_inertia_share_box_orientation(self):
        link = expand_nira().find("link[@name='right_mid_femur_link']")
        for kind in ('visual', 'inertial'):
            self.assertEqual(link.find(f'{kind}/origin').attrib,
                             link.find('collision/origin').attrib)


if __name__ == '__main__':
    unittest.main()
