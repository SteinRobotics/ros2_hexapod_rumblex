"""Check the closed support rim underneath Nira's deck opening."""

import unittest

from build123d import Face, offset

from robot_nira import torso_layer_2, torso_layer_2_addon as addon


class NiraTorsoAddonTests(unittest.TestCase):
    def test_ring_follows_actual_deck_opening_and_stays_closed_at_cables(self):
        deck = torso_layer_2.build_surface()
        opening = max((Face(wire) for face in deck.faces() for wire in face.inner_wires()),
                      key=lambda face: face.area)
        surface = addon.build_surface()
        self.assertTrue(surface.is_valid)
        self.assertEqual(len(surface.faces()), 1)
        self.assertEqual(len(surface.faces()[0].inner_wires()), 1)
        actual_outline = Face(surface.faces()[0].outer_wire())
        expected_outline = offset(opening, amount=addon.FINGER_WIDTH / 2)
        self.assertAlmostEqual(actual_outline.area, expected_outline.area)
        self.assertAlmostEqual((actual_outline & expected_outline).area, actual_outline.area)
        # Even where cables remove the support ledge, the outer half stays continuous.
        rim = expected_outline - opening
        self.assertAlmostEqual((surface & rim).area, rim.area)
        support = surface & opening
        self.assertGreater(support.area, 0)
        self.assertLess(support.area, (opening - offset(opening, amount=-addon.FINGER_WIDTH / 2)).area)

    def test_model_is_one_valid_one_mm_thick_solid(self):
        model = addon.build_model(addon.build_surface())
        self.assertTrue(model.is_valid)
        self.assertEqual(len(model.solids()), 1)
        self.assertAlmostEqual(model.bounding_box().size.Z, addon.THICKNESS)


if __name__ == '__main__':
    unittest.main()
