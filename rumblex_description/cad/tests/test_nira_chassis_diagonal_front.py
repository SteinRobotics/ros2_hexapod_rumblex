"""Check standalone front braces and their removal from the torso."""

import unittest

from build123d import Box, Pos

from robot_nira import (
    chassis_common, chassis_diagonal_back, chassis_diagonal_front,
    torso_common, torso_layer_2,
)
from robot_nira.torso_layout import (
    DIAGONAL_PLATE_HEIGHT, LAYER_2_BOTTOM, LAYER_3_BOTTOM,
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

    def test_only_rear_diagonal_braces_are_assembled(self):
        plates = chassis_diagonal_back.build_diagonal_plates(LAYER_2_BOTTOM, LAYER_3_BOTTOM)
        self.assertEqual({plate.label for plate in plates}, {
            'chassis_diagonal_back_left', 'chassis_diagonal_back_right',
        })
        for plate in plates:
            self.assertTrue(plate.is_valid)
            self.assertEqual(len(plate.solids()), 1)

    def test_front_diagonal_cutouts_are_filled(self):
        deck = torso_layer_2.build_model(torso_layer_2.build_surface())
        self.assertTrue(deck.is_valid)
        self.assertEqual(len(deck.solids()), 1)
        for location in torso_common.front_diagonal_slots_locations:
            for offset in torso_common.front_diagonal_tab_offsets:
                with self.subTest(location=location, offset=offset):
                    slot = location * Pos(offset, 0, torso_common.THICKNESS / 2) * Box(
                        torso_common.front_diagonal_tab_width,
                        torso_common.rectangle_slots_height, torso_common.THICKNESS)
                    self.assertAlmostEqual((slot & deck).volume, slot.volume)


if __name__ == '__main__':
    unittest.main()
