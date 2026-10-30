"""Headless description checks; run with the ROS environment sourced."""
from pathlib import Path
import math
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import xacro
import xacro.substitution_args
import yaml

ROOT = Path(__file__).resolve().parents[2]


def expand(mesh=False, sim=False):
    path = ROOT / 'rumblex_description/urdf' / ('nira_mesh.urdf.xacro' if mesh else 'nira.urdf.xacro')
    with patch('xacro.substitution_args._eval_find', side_effect=lambda name: str(ROOT / name)):
        return ET.fromstring(xacro.process_file(str(path), mappings={'use_sim': str(sim).lower()}).toxml())


class NiraDescriptionTests(unittest.TestCase):
    def test_imu_measurement_frame_is_upright(self):
        for mesh in (False, True):
            model = expand(mesh, True)
            joint = model.find("joint[@name='imu_joint']")
            self.assertEqual(joint.find('parent').get('link'), 'base_link')
            self.assertEqual(joint.find('origin').get('xyz'), '0 -0.002 0.0185')
            # Sensor Z points upward; the CAD envelope flip is not a measurement-axis transform.
            roll, pitch, yaw = map(float, joint.find('origin').get('rpy').split())
            self.assertAlmostEqual(roll, 0)
            self.assertAlmostEqual(pitch, 0)
            self.assertAlmostEqual(yaw, -math.pi / 2)
            if not mesh:
                visual = model.find("link[@name='imu_link']/visual/origin")
                self.assertEqual(visual.get('xyz'), '0 0 -0.0075')
                self.assertAlmostEqual(float(visual.get('rpy').split()[0]), math.pi)
            sensor = model.find("gazebo[@reference='imu_link']/sensor")
            self.assertEqual(sensor.get('type'), 'imu')
            self.assertEqual(sensor.findtext('gz_frame_id'), 'imu_link')
            self.assertEqual(sensor.findtext('topic'), 'imu/data')
            self.assertEqual(sensor.findtext('imu/orientation_reference_frame/localization'), 'ENU')
            self.assertIsNone(expand(mesh).find(".//sensor[@type='imu']"))

    @unittest.skipUnless(shutil.which('gz'), 'Gazebo CLI unavailable')
    def test_imu_survives_fixed_joint_reduction(self):
        for mesh in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'nira.urdf'
                ET.ElementTree(expand(mesh, True)).write(path)
                result = subprocess.run(['gz', 'sdf', '-p', str(path)], check=True,
                                        capture_output=True, text=True)
            sdf = ET.fromstring(result.stdout)
            sensor = sdf.find("model/link[@name='base_link']/sensor[@name='imu_bno055']")
            self.assertIsNotNone(sensor)
            self.assertEqual(sensor.findtext('gz_frame_id'), 'imu_link')
            pose = list(map(float, sensor.findtext('pose').split()))
            for actual, expected in zip(pose[:3], [0, -0.002, 0.0185]):
                self.assertAlmostEqual(actual, expected)
            self.assertAlmostEqual(pose[3], 0, places=5)
            self.assertAlmostEqual(pose[4], 0)
            self.assertAlmostEqual(pose[5], -math.pi / 2, places=5)

    def test_lidar_is_a_horizontal_scan_at_the_cad_mount(self):
        config = yaml.safe_load((ROOT / 'rumblex_lidar_2d/config/nira/lidar.yaml').read_text())
        driver = config['ydlidar_ros2_driver_node']['ros__parameters']
        for mesh in (False, True):
            self.assertIsNone(expand(mesh).find('.//sensor'))
            model = expand(mesh, True)
            mount = model.find("joint[@name='lidar_joint']")
            self.assertEqual(mount.find('parent').get('link'), 'base_link')
            self.assertEqual(mount.find('origin').get('xyz'), '0.093 0 0.0933')
            sensor = model.find("gazebo[@reference='lidar_link']/sensor")
            self.assertEqual(sensor.get('type'), 'gpu_lidar')
            self.assertEqual(sensor.findtext('gz_frame_id'), driver['frame_id'])
            self.assertEqual(sensor.findtext('topic'), 'lidar_scan')
            self.assertEqual(float(sensor.findtext('update_rate')), driver['frequency'])
            scan = sensor.find('lidar/scan')
            self.assertEqual(int(scan.findtext('horizontal/samples')), 400)
            for bound in ('min', 'max'):
                self.assertAlmostEqual(float(scan.findtext(f'horizontal/{bound}_angle')),
                                       math.radians(driver[f'angle_{bound}']))
                self.assertEqual(float(scan.findtext(f'vertical/{bound}_angle')), 0)
                self.assertEqual(float(sensor.findtext(f'lidar/range/{bound}')),
                                 driver[f'range_{bound}'])
            self.assertEqual(int(scan.findtext('vertical/samples')), 1)

    def test_primitive_visual_clears_scan_plane_and_shows_lidar(self):
        model = expand()
        visual = model.find("link[@name='base_link']/visual")
        top = float(visual.find('origin').get('xyz').split()[2])
        top += float(visual.find('geometry/box').get('size').split()[2]) / 2
        self.assertLessEqual(top, 0.067)
        self.assertEqual(len(model.findall("link[@name='lidar_link']/visual")), 2)
        # The mesh torso already contains the scanner housing.
        self.assertFalse(expand(True).findall("link[@name='lidar_link']/visual"))

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

    def test_visual_materials_match_nira_parts(self):
        for mesh in (False, True):
            model = expand(mesh)
            materials = {link.get('name'): link.find('visual/material').get('name')
                         for link in model.findall('link') if link.find('visual') is not None}
            self.assertEqual(materials['base_link'], 'creamy_white')
            for prefix in ('right_front', 'right_mid', 'right_back',
                           'left_front', 'left_mid', 'left_back'):
                self.assertEqual(materials[f'{prefix}_coxa_link'], 'dark_gray')
                self.assertEqual(materials[f'{prefix}_femur_link'], 'dark_gray')
                self.assertEqual(materials[f'{prefix}_tibia_link'], 'creamy_white')


if __name__ == '__main__':
    unittest.main()
