"""Check front brace dimensions, servo alignment, and deck engagement."""

import math
import unittest

from build123d import Box, Pos, Rot

from robot_nira import (
    chassis_common, chassis_diagonal_back, chassis_diagonal_front,
    lidar_interface_housing, torso_common, torso_layer_2,
)
from robot_nira.lidar_layout import LIDAR_ASSEMBLY_X, LIDAR_Y
from robot_nira.torso_layout import (
    DIAGONAL_PLATE_HEIGHT, LAYER_2_BOTTOM, LAYER_2_TOP, LAYER_3_BOTTOM,
)


class NiraFrontDiagonalTests(unittest.TestCase):
    def test_brace_is_24_mm_wide_and_centered_on_its_tabs(self):
        plate = chassis_diagonal_front.build_model(DIAGONAL_PLATE_HEIGHT)
        self.assertTrue(plate.is_valid)
        self.assertEqual(len(plate.solids()), 1)
        bounds = plate.bounding_box()
        self.assertAlmostEqual(bounds.min.X, -12)
        self.assertAlmostEqual(bounds.max.X, 12)
        self.assertAlmostEqual(bounds.max.Y, 10)
        fingers = plate & Pos(0, chassis_common.TAB_DEPTH / 2,
                              chassis_common.THICKNESS / 2) * Box(
                                  30, chassis_common.TAB_DEPTH, chassis_common.THICKNESS)
        self.assertEqual(len(fingers.solids()), 2)
        self.assertAlmostEqual(fingers.bounding_box().center().X, 0)

    def test_front_pair_center_on_servo_axes_and_fit_deck(self):
        deck = Pos(Z=LAYER_2_BOTTOM) * torso_layer_2.build_model(torso_layer_2.build_surface())
        housing = (Pos(LIDAR_ASSEMBLY_X, LIDAR_Y, LAYER_2_TOP) * Rot(Z=180)
                   * lidar_interface_housing.build_model())
        plates = {part.label: part for part in chassis_diagonal_back.build_diagonal_plates(
            LAYER_2_BOTTOM, LAYER_3_BOTTOM) if 'front' in part.label}
        self.assertEqual(len(plates), 2)
        for location in torso_common.front_diagonal_slots_locations:
            side = 'left' if location.position.Y > 0 else 'right'
            servo = torso_common.SERVO_CUTOUT_CONFIGS[
                'right_up' if side == 'left' else 'right_down']
            angle = math.radians(location.orientation.Z)
            with self.subTest(side=side):
                self.assertAlmostEqual(
                    (location.position.X - servo.offset_x) * math.cos(angle)
                    + (location.position.Y - servo.offset_y) * math.sin(angle), 0)
                plate = plates[f'chassis_diagonal_front_{side}']
                self.assertTrue(plate.is_valid)
                self.assertEqual(len(plate.solids()), 1)
                overlap = plate & deck
                self.assertLess(overlap.volume if overlap else 0, 1e-6)
                fingers = plate & Pos(Z=(LAYER_2_BOTTOM + LAYER_2_TOP) / 2) * Box(
                    300, 200, torso_common.THICKNESS)
                self.assertEqual(len(fingers.solids()), 2)
                self.assertAlmostEqual(fingers.volume, 2 * torso_common.front_diagonal_tab_width
                                       * chassis_common.THICKNESS * torso_common.THICKNESS)
                self.assertGreater(plate.distance_to(housing), 1.0)


if __name__ == '__main__':
    unittest.main()
