"""Check Nox mesh resources, joint tree and simulation configuration."""
from pathlib import Path
import math
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import xacro
import xacro.substitution_args

ROOT = Path(__file__).resolve().parents[2]


def expand(mesh=True, sim=False):
    path = ROOT / 'rumblex_description/urdf' / ('nox_mesh.urdf.xacro' if mesh else 'nox.urdf.xacro')
    with patch('xacro.substitution_args._eval_find', side_effect=lambda name: str(ROOT / name)):
        return ET.fromstring(xacro.process_file(str(path), mappings={'use_sim': str(sim).lower()}).toxml())


class NoxDescriptionTests(unittest.TestCase):
    def test_gazebo_resource_export_resolves_mesh_uris(self):
        share = ROOT / 'rumblex_description'
        manifest = ET.parse(share / 'package.xml')
        export = manifest.find('export/gazebo_ros')
        self.assertIsNotNone(export)
        resource_root = Path(export.get('gazebo_model_path').replace('${prefix}', str(share)))
        for mesh in expand(sim=True).findall('.//mesh'):
            relative = mesh.get('filename').removeprefix('package://')
            self.assertTrue((resource_root / relative).is_file(), relative)

    def test_mesh_resources_and_tree(self):
        model = expand()
        for mesh in model.findall('.//mesh'):
            self.assertTrue((ROOT / mesh.get('filename').removeprefix('package://')).is_file())
            self.assertEqual(mesh.get('scale'), '0.001 0.001 0.001')
        joints = model.findall('joint')
        links = {link.get('name') for link in model.findall('link')}
        self.assertEqual(sum(j.get('type') == 'revolute' for j in joints), 20)
        children = [j.find('child').get('link') for j in joints]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(links - set(children), {'base_link'})
        reached = {'base_link'}
        for _ in joints:
            reached.update(j.find('child').get('link') for j in joints
                           if j.find('parent').get('link') in reached)
        self.assertEqual(reached, links)

    def test_leg_names_match_robot_sides_and_mount_headings(self):
        for mesh in (False, True):
            model = expand(mesh)
            for side, sign in (('right', -1), ('left', 1)):
                for location in ('front', 'mid', 'back'):
                    name = f'{side}_{location}_coxa_joint'
                    with self.subTest(mesh=mesh, joint=name):
                        joint = model.find(f"joint[@name='{name}']")
                        _, y, _ = map(float, joint.find('origin').get('xyz').split())
                        _, _, yaw = map(float, joint.find('origin').get('rpy').split())
                        self.assertGreater(sign * y, 0.0)
                        # Primitive Nox points local +Y outward; CAD legs use +X.
                        heading = yaw + (math.pi / 2 if not mesh and model.get('name') == 'nox' else 0)
                        expected = sign * math.radians({'front': 45, 'mid': 90, 'back': 135}[location])
                        self.assertAlmostEqual(math.sin(heading), math.sin(expected), places=6)
                        self.assertAlmostEqual(math.cos(heading), math.cos(expected), places=6)

    def test_head_yaw_rotates_about_positive_torso_z(self):
        for mesh in (False, True):
            joint = expand(mesh).find("joint[@name='head_yaw_joint']")
            self.assertEqual(joint.find('parent').get('link'), 'base_link')
            roll, pitch, yaw = map(float, joint.find('origin').get('rpy').split())
            ax, ay, az = map(float, joint.find('axis').get('xyz').split())
            # Rotate the local axis by the joint's ZYX mounting rotation.
            x = ax
            y = math.cos(roll) * ay - math.sin(roll) * az
            z = math.sin(roll) * ay + math.cos(roll) * az
            x, z = math.cos(pitch) * x + math.sin(pitch) * z, -math.sin(pitch) * x + math.cos(pitch) * z
            x, y = math.cos(yaw) * x - math.sin(yaw) * y, math.sin(yaw) * x + math.cos(yaw) * y
            self.assertAlmostEqual(x, 0.0)
            self.assertAlmostEqual(y, 0.0)
            self.assertAlmostEqual(z, 1.0)

    def test_simulation_configuration(self):
        self.assertIsNone(expand().find('ros2_control'))
        model = expand(sim=True)
        self.assertEqual({j.get('name') for j in model.findall('ros2_control/joint')},
                         {j.get('name') for j in model.findall('joint') if j.get('type') == 'revolute'})
        controller = Path(model.find('gazebo/plugin/parameters').text)
        self.assertTrue(controller.is_file())
        self.assertEqual(controller.parent.name, 'nox')

    def test_lidar_frame_follows_head(self):
        model = expand()
        joint = model.find("joint[@name='lidar_joint']")
        self.assertEqual(joint.get('type'), 'fixed')
        self.assertEqual(joint.find('parent').get('link'), 'head_pitch_link')
        self.assertEqual(joint.find('child').get('link'), 'lidar_link')
        self.assertIsNotNone(model.find("link[@name='lidar_link']"))


if __name__ == '__main__':
    unittest.main()
