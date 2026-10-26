"""Check cover ventilation dimensions and clearance from the notched outline."""

import math
import unittest

from build123d import Face
from robot_nira import torso_layer_2_cover as cover


class NiraTorsoCoverTests(unittest.TestCase):
    def test_vent_holes_are_four_mm_on_grid_and_twenty_mm_from_all_edges(self):
        plate = cover.build_opening_outline()
        plate -= cover.build_cable_cutouts(plate)
        surface = cover.build_surface()
        self.assertTrue(surface.is_valid)
        self.assertEqual(len(surface.faces()), 1)
        vents = surface.faces()[0].inner_wires()
        self.assertGreater(len(vents), 0)
        self.assertAlmostEqual(Face(surface.faces()[0].outer_wire()).area,
                               Face(plate.faces()[0].outer_wire()).area)
        for vent in vents:
            hole = Face(vent)
            self.assertAlmostEqual(hole.area, math.pi * 2**2)
            center = hole.bounding_box().center()
            self.assertAlmostEqual(center.X / cover.VENT_PITCH,
                                   round(center.X / cover.VENT_PITCH))
            self.assertAlmostEqual(center.Y / cover.VENT_PITCH,
                                   round(center.Y / cover.VENT_PITCH))
            for boundary in plate.wires():
                self.assertGreaterEqual(vent.distance_to(boundary), 20 - 1e-6)
        self.assertAlmostEqual(plate.area - surface.area, len(vents) * math.pi * 2**2)
        model = cover.build_model(surface)
        self.assertTrue(model.is_valid)
        self.assertEqual(len(model.solids()), 1)
        self.assertAlmostEqual(model.bounding_box().size.Z, cover.THICKNESS)


if __name__ == '__main__':
    unittest.main()
