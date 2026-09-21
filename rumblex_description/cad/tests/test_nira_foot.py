"""Nominal outer armor engagement and retained Nira foot interfaces."""

import unittest

from build123d import Box, Pos

from robot_nira import foot_back, foot_front, foot_outer
from robot_nira import foot_common as layout
from robot_nira.assembly_foot import build_assembly
from robot_nira.assembly_tibia import build_assembly as build_tibia


class NiraFootTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.foot = build_assembly()
        cls.outer = cls.foot.children[-1]

    def test_plates_are_single_valid_solids(self):
        for module in (foot_front, foot_back, foot_outer):
            with self.subTest(plate=module.__name__):
                surface = module.build_surface()
                self.assertEqual(len(surface.faces()), 1)
                part = module.build_model(surface)
                self.assertTrue(part.is_valid)
                self.assertEqual(len(part.solids()), 1)

    def test_outer_plate_clears_existing_components(self):
        for other in self.foot.children[:-1]:
            overlap = self.outer & other
            self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_six_tabs_fill_slots_and_finish_flush(self):
        t = layout.THICKNESS
        bounds = self.outer.bounding_box()
        self.assertAlmostEqual(bounds.min.Z, 0)
        self.assertAlmostEqual(bounds.max.Z, layout.OUTER_PLATE_HEIGHT)
        for z in (t / 2, layout.OUTER_PLATE_HEIGHT - t / 2):
            tabs = self.outer & (Pos(50, layout.OUTER_PLATE_Y, z) * Box(130, 5, t))
            self.assertEqual(len(tabs.solids()), 3)
            self.assertAlmostEqual(tabs.volume, 3 * layout.OUTER_TAB_WIDTH * t * t)
            for x in layout.OUTER_TAB_X:
                slot = Pos(x, layout.OUTER_PLATE_Y, z) * Box(layout.OUTER_TAB_WIDTH, t, t)
                self.assertAlmostEqual((self.outer & slot).volume, slot.volume)

    def test_outer_plate_clears_tibia_servo(self):
        tibia = build_tibia()
        servo, foot = tibia.children
        overlap = servo & foot.children[-1]
        self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_mounting_pattern_is_retained(self):
        self.assertEqual([(h['x'], h['y'], h['radius']) for h in layout.FOOT_MOUNT_HOLES],
                         [(84.75, -8.25, 1.25), (52.75, -19.75, 1.25), (-14.25, -19.75, 1.25)])
        self.assertEqual(layout.TIP_SLOTS, [(88.25, -3.75, 3.0, 4.0), (88.25, -12.75, 3.0, 4.0)])


if __name__ == '__main__':
    unittest.main()
