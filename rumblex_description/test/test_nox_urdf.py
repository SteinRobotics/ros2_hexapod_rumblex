"""Check Nox mesh resources, joint tree and simulation configuration."""
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import xacro
import xacro.substitution_args

ROOT = Path(__file__).resolve().parents[2]


def expand(sim=False):
    path = ROOT / 'rumblex_description/urdf/nox_mesh.urdf.xacro'
    with patch('xacro.substitution_args._eval_find', side_effect=lambda name: str(ROOT / name)):
        return ET.fromstring(xacro.process_file(str(path), mappings={'use_sim': str(sim).lower()}).toxml())


class NoxDescriptionTests(unittest.TestCase):
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

    def test_simulation_configuration(self):
        self.assertIsNone(expand().find('ros2_control'))
        model = expand(True)
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
