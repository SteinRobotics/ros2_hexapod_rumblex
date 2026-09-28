"""Mechanical relationships shared by Nira parts and assemblies."""
import unittest
from unittest.mock import patch

from build123d import Cylinder, Pos, Vector

from common import servo_simplified as servo
from robot_nira import (assembly_body, assembly_body_with_servos, assembly_femur,
                        body_layer_0, body_layout, board_layout, foot_common,
                        foot_connection, vendor_brackets)


class NiraMountingFrameTests(unittest.TestCase):
    def test_vendor_bracket_bores_align_with_servo_rows(self):
        side = vendor_brackets.build_side_bracket()
        _, centers = max(vendor_brackets.mounting_faces(side, 1.15, 2, (0, -1, 0)),
                         key=lambda item: vendor_brackets.midpoint(item[1]).Y)
        for actual, z in zip(sorted(centers, key=lambda p: p.Z),
                             (servo.HOLE_Z_LOW, servo.HOLE_Z_HIGH)):
            expected = Vector(servo.HOLE_X, servo.BODY_Y + servo.CASE_FLANGE_THICKNESS, z)
            self.assertLess((actual - expected).length, 1e-5)
        bottom = vendor_brackets.build_bottom_bracket()
        _, centers = min(vendor_brackets.mounting_faces(bottom, servo.M2_R, 2, (0, 1, 0)),
                         key=lambda item: vendor_brackets.midpoint(item[1]).Y)
        for actual, x in zip(sorted(centers, key=lambda p: p.X), (-servo.HOLE_X, servo.HOLE_X)):
            expected = Vector(x, -servo.CASE_FLANGE_THICKNESS, servo.HOLE_Z_LOW)
            self.assertLess((actual - expected).length, 1e-5)

    def test_vendor_registration_does_not_depend_on_step_origin(self):
        builders = (vendor_brackets.build_side_bracket, vendor_brackets.build_bottom_bracket)
        originals = [build() for build in builders]
        importer = vendor_brackets.import_step
        try:
            vendor_brackets._template.cache_clear()
            with patch.object(vendor_brackets, 'import_step',
                              side_effect=lambda path: Pos(123, -76, 45) * importer(path)):
                for original, build in zip(originals, builders):
                    shifted = build()
                    self.assertLess((original.bounding_box().center()
                                     - shifted.bounding_box().center()).length, 1e-5)
                    for name in ('servo_mount', 'plate_mount'):
                        self.assertLess((original.joints[name].location.position
                                         - shifted.joints[name].location.position).length, 1e-5)
        finally:
            vendor_brackets._template.cache_clear()

    def test_femur_plate_joint_seats_on_vendor_bracket(self):
        femur = assembly_femur.build_assembly()
        _, bottom, inclined = femur.children
        mount = bottom.joints['plate_mount'].location
        other = inclined.joints['plate_mount'].location
        self.assertLess((mount.position - other.position).length, 1e-6)
        self.assertGreater(mount.z_axis.direction.dot(other.z_axis.direction), 1 - 1e-6)
        self.assertLess(bottom.distance_to(inclined), 1e-5)
        # Connecting an instance must not move the cached bracket template.
        fresh = vendor_brackets.build_bottom_bracket()
        self.assertAlmostEqual(fresh.joints['servo_mount'].location.position.Z, servo.HOLE_Z_LOW)


class NiraBodyLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.body = assembly_body.build_assembly()
        cls.parts = {part.label: part for part in cls.body.children}

    def test_layers_and_cover_follow_shared_mounting_elevations(self):
        for index in range(5):
            self.assertAlmostEqual(self.parts[f'body_layer_{index}'].bounding_box().min.Z,
                                   getattr(body_layout, f'LAYER_{index}_BOTTOM'))
        cover = self.parts['body_layer_2_cover']
        self.assertAlmostEqual(cover.bounding_box().min.Z, body_layout.COVER_BOTTOM)
        for name in ('battery_left', 'battery_right'):
            self.assertLess(self.parts[name].distance_to(cover), 1e-6)

    def test_body_toes_seat_on_plate_underside(self):
        plate = self.parts['body_layer_0']
        for i, (x, y) in enumerate(body_layer_0.TOE_MOUNTING_POSITIONS):
            toe = self.parts[f'toe_{i}']
            bounds = toe.bounding_box()
            self.assertAlmostEqual(bounds.center().X, x)
            self.assertAlmostEqual(bounds.center().Y, y)
            self.assertAlmostEqual(bounds.max.Z, plate.bounding_box().min.Z)
            self.assertLess(toe.distance_to(plate), 1e-6)
            overlap = toe & plate
            self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_board_spacers_align_with_base_holes(self):
        plate = self.parts['body_layer_0']
        z = plate.bounding_box().center().Z
        for mount in board_layout.BOARD_MOUNTS:
            board = assembly_body_with_servos.place_board(mount.module, mount.placement)
            for post in board.children[1:]:
                placed = board.location * post
                center = placed.bounding_box().center()
                probe = Pos(center.X, center.Y, z) * Cylinder(
                    mount.module.cfg.hole_diameter / 2 - 1e-4, plate.bounding_box().size.Z)
                overlap = probe & plate
                self.assertLess(overlap.volume if overlap else 0, 1e-6, mount.module.__name__)
                self.assertLess(placed.distance_to(plate), 1e-6)

    def test_connection_spans_cheek_outer_faces(self):
        connection = foot_connection.build_model(foot_connection.build_surface())
        self.assertAlmostEqual(connection.bounding_box().size.X,
                               foot_common.FOOT_SPACER_OVERALL_LENGTH + 2 * foot_common.THICKNESS)


if __name__ == '__main__':
    unittest.main()
