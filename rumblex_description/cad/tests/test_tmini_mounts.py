"""Check the T-mini mounting interfaces against datasheet V1.1, Fig. 2."""

import unittest

from build123d import Box, Cylinder, Pos

from robot_nira.lidar_ydlidar_tmini import build_model


class TminiMountTests(unittest.TestCase):
    def test_front_socket_is_recessed_with_four_contacts(self):
        model = build_model()
        self.assertTrue(model.is_valid)
        self.assertEqual(len(model.solids()), 1)
        joint = model.joints['connector'].location
        self.assertLess((joint.position - (0, -19.0, 4.65)).length, 1e-7)
        self.assertLess((joint.z_axis.direction - (0, -1, 0)).length, 1e-7)
        # Recess, cavity between contacts, surrounding socket rim and housing.
        for x, y, z, solid in ((0, -19.2, 4.65, False),
                               (0, -18.5, 4.65, False),
                               (3.9, -18.5, 4.65, True),
                               (0, -19.2, 2.3, True)):
            probe = Pos(x, y, z) * Box(0.05, 0.05, 0.05)
            self.assertAlmostEqual((model & probe).volume, probe.volume if solid else 0)
        for x in (-1.875, -0.625, 0.625, 1.875):
            probe = Pos(x, -18.5, 4.65) * Box(0.1, 0.1, 0.1)
            self.assertAlmostEqual((model & probe).volume, probe.volume)
        self.assertAlmostEqual(model.bounding_box().size.Y, 38.6)

    def test_drawing_bores_and_assembly_joints(self):
        model = build_model()
        self.assertTrue(model.is_valid)
        self.assertEqual(len(model.solids()), 1)
        self.assertAlmostEqual(model.bounding_box().size.Z, 33.9)
        for index, (x, y) in enumerate(((-14.1, 14.1), (14.1, -14.1)), 1):
            self.assertLess(
                (model.joints[f'mount_{index}'].location.position - (x, y, 0)).length,
                1e-7,
            )
            # Full bore, bottom counterbore, web between counterbores, top recess.
            for z, radius in ((10.575, 1.25), (6.15, 2.4), (14, 1.25), (18.55, 1.79)):
                depth = 21.15 if z == 10.575 else 0.1
                probe = Pos(x, y, z) * Cylinder(radius - 0.001, depth)
                self.assertLess((model & probe).volume, 1e-6)
                if depth == 0.1:
                    rim = Pos(x + radius + 0.05, y, z) * Cylinder(0.02, 0.05)
                    self.assertAlmostEqual((model & rim).volume, rim.volume)

        # The third hole is blind: preserve material immediately above 7.2 mm.
        bore = Pos(15.7, 15.7, 3.6) * Cylinder(1.249, 7.2)
        self.assertLess((model & bore).volume, 1e-6)
        cap = Pos(15.7, 15.7, 7.3) * Cylinder(1, 0.1)
        self.assertAlmostEqual((model & cap).volume, cap.volume)


if __name__ == '__main__':
    unittest.main()
