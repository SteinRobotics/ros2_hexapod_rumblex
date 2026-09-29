"""Headless description checks; run with the ROS environment sourced."""
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import xacro
import xacro.substitution_args

ROOT = Path(__file__).resolve().parents[2]


def expand(mesh=False, sim=False):
    path = ROOT / 'rumblex_description/urdf' / ('nira_mesh.urdf.xacro' if mesh else 'nira.urdf.xacro')
    with patch('xacro.substitution_args._eval_find', side_effect=lambda name: str(ROOT / name)):
        return ET.fromstring(xacro.process_file(str(path), mappings={'use_sim': str(sim).lower()}).toxml())


class NiraDescriptionTests(unittest.TestCase):
    def test_variants_share_complete_joint_tree(self):
        primitive, mesh = expand(), expand(True)
        self.assertEqual([ET.tostring(j) for j in primitive.findall('joint')],
                         [ET.tostring(j) for j in mesh.findall('joint')])
        for model in (primitive, mesh):
            links = {link.get('name') for link in model.findall('link')}
            joints = model.findall('joint')
            self.assertEqual(len([j for j in joints if j.get('type') == 'revolute']), 20)
            children = [j.find('child').get('link') for j in joints]
            self.assertEqual(len(children), len(set(children)))
            self.assertEqual(links - set(children), {'base_link'})
            reachable = {'base_link'}
            for _ in joints:
                reachable.update(j.find('child').get('link') for j in joints
                                 if j.find('parent').get('link') in reachable)
            self.assertEqual(reachable, links)
        self.assertFalse(primitive.findall('.//mesh'))
        for element in mesh.findall('.//mesh'):
            self.assertTrue((ROOT / element.get('filename').removeprefix('package://')).is_file())
            self.assertEqual(element.get('scale'), '0.001 0.001 0.001')

    def test_simulation_uses_nira_controller_and_all_moving_joints(self):
        for mesh in (False, True):
            self.assertIsNone(expand(mesh).find('ros2_control'))
            model = expand(mesh, True)
            actual = {j.get('name') for j in model.findall('ros2_control/joint')}
            expected = {j.get('name') for j in model.findall('joint') if j.get('type') == 'revolute'}
            self.assertEqual(actual, expected)
            controller = Path(model.find('gazebo/plugin/parameters').text)
            self.assertTrue(controller.is_file())
            self.assertEqual(controller.parent.name, 'nira')


if __name__ == '__main__':
    unittest.main()
