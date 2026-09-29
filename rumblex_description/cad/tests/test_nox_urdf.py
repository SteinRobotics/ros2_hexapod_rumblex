"""Compare articulated URDF mesh placement with the actual Nox CAD joints.

Requires the CAD virtualenv and a sourced ROS environment (for xacro).
"""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

CAD = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CAD))
spec = importlib.util.spec_from_file_location('description_checks', CAD.parent / 'test/test_nox_urdf.py')
description = importlib.util.module_from_spec(spec)
spec.loader.exec_module(description)

from robot_nox import assembly_coxa, assembly_femur, assembly_tibia, assembly_head
from robot_nox import assembly_torso_with_servos, torso_common
from common import servo_simplified


def transform(element):
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        matrix[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return matrix


def cad_transform(location):
    matrix = np.eye(4)
    tr = location.wrapped.Transformation()
    matrix[:3, :3] = [[tr.Value(i, j) for j in range(1, 4)] for i in range(1, 4)]
    matrix[:3, 3] = [tr.Value(i, 4) / 1000 for i in range(1, 4)]
    return matrix


def link_transforms(model, angle):
    frames = {'base_link': np.eye(4)}
    pending = list(model.findall('joint'))
    while pending:
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            rotation = np.eye(4)
            if joint.get('type') == 'revolute':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                rotation[:3, :3] = Rotation.from_rotvec(axis * np.deg2rad(angle)).as_matrix()
            frames[joint.find('child').get('link')] = frames[parent] @ transform(joint.find('origin')) @ rotation
            pending.remove(joint)
    return frames


class NoxCadDescriptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = description.expand()
        cls.servo = servo_simplified.build_model()
        cls.coxa = assembly_coxa.build_assembly()
        cls.femur = assembly_femur.build_assembly()
        cls.tibia = assembly_tibia.build_assembly()
        cls.head = assembly_head.build_assembly()

    def assert_mesh_placement(self, name, part, frames):
        visual = self.model.find(f"link[@name='{name}']/visual/origin")
        np.testing.assert_allclose(frames[name] @ transform(visual), cad_transform(part.location), atol=1e-9)

    def test_exported_mesh_bounds_match_cad(self):
        triangle = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        for name in ('coxa', 'femur', 'tibia', 'head'):
            with self.subTest(part=name):
                path = CAD.parent / 'meshes/nox' / f'assembly_{name}.stl'
                vertices = np.fromfile(path, dtype=triangle, offset=84)['vertices'].reshape(-1, 3)
                part = getattr(self, name)
                bounds = (part.location.inverse() * part).bounding_box()
                np.testing.assert_allclose(vertices.min(axis=0), tuple(bounds.min), atol=0.1)
                np.testing.assert_allclose(vertices.max(axis=0), tuple(bounds.max), atol=0.1)

    def test_zero_and_articulated_meshes_match_cad(self):
        names = {'right_front': 'right_up', 'right_mid': 'center_up', 'right_back': 'left_up',
                 'left_front': 'right_down', 'left_mid': 'center_down', 'left_back': 'left_down'}
        for angle in (0, 12):
            frames = link_transforms(self.model, angle)
            for prefix, mount in names.items():
                with self.subTest(angle=angle, leg=prefix):
                    servo = assembly_torso_with_servos.servo_location(torso_common.SERVO_CUTOUT_CONFIGS[mount]) * self.servo
                    servo.joints['rotation'].connect_to(self.coxa.joints['body_to_coxa_fixed'], angle=angle)
                    self.coxa.joints['coxa_to_femur_fixed'].connect_to(self.femur.joints['femur_to_coxa_revolute'], angle=(-angle) % 360)
                    self.femur.joints['femur_to_tibia_fixed'].connect_to(self.tibia.joints['tibia_to_femur_revolute'], angle=180 + angle)
                    for segment in ('coxa', 'femur', 'tibia'):
                        self.assert_mesh_placement(f'{prefix}_{segment}_link', getattr(self, segment), frames)
                    foot = self.tibia.children[1]
                    toe = foot.children[1]
                    np.testing.assert_allclose(frames[f'{prefix}_foot'],
                                               cad_transform(self.tibia.location * foot.location * toe.location), atol=1e-9)
            servo = assembly_torso_with_servos.servo_location(torso_common.SERVO_CUTOUT_CONFIGS['head']) * self.servo
            servo.joints['rotation'].connect_to(self.coxa.joints['body_to_coxa_fixed'], angle=angle)
            self.coxa.joints['coxa_to_femur_fixed'].connect_to(self.head.joints['pitch'], angle=300 + angle)
            self.assert_mesh_placement('head_yaw_link', self.coxa, frames)
            self.assert_mesh_placement('head_pitch_link', self.head, frames)


if __name__ == '__main__':
    unittest.main()
