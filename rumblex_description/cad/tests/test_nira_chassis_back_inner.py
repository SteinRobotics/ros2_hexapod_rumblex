"""Rear backing plate dimensions, opening alignment, and assembly clearance."""
import math
import unittest

from build123d import GeomType
from robot_nira import assembly_torso, chassis_back_inner, torso_layout


class ChassisBackInnerTests(unittest.TestCase):
    def test_dimensions_and_through_hole(self):
        plate = chassis_back_inner.build_model()
        self.assertTrue(plate.is_valid)
        self.assertEqual(len(plate.solids()), 1)
        size = plate.bounding_box().size
        self.assertAlmostEqual(size.X, 40)
        self.assertAlmostEqual(size.Y, 31)
        self.assertAlmostEqual(size.Z, 3)
        self.assertAlmostEqual(plate.volume, (40 * 31 - math.pi * 10**2) * 3)
        for edge in plate.edges().filter_by(GeomType.CIRCLE):
            self.assertAlmostEqual(edge.radius, 10)
            self.assertAlmostEqual(edge.arc_center.X, 0)
            self.assertAlmostEqual(edge.arc_center.Y, 0)

    def test_inner_seating_and_assembly_clearance(self):
        body = assembly_torso.build_assembly()
        parts = {part.label: part for part in body.children}
        plate = parts["chassis_back_inner"]
        rear = parts["chassis_back"]
        bounds = plate.bounding_box()
        self.assertAlmostEqual(bounds.min.X, rear.bounding_box().max.X)
        self.assertAlmostEqual(bounds.min.Z, parts["torso_layer_1"].bounding_box().max.Z)
        self.assertAlmostEqual(bounds.max.Z, torso_layout.LAYER_2_BOTTOM - 1)
        rear_hole = list(rear.edges().filter_by(GeomType.CIRCLE))[0]
        for edge in plate.edges().filter_by(GeomType.CIRCLE):
            self.assertAlmostEqual(edge.arc_center.Y, rear_hole.arc_center.Y)
            self.assertAlmostEqual(edge.arc_center.Z, rear_hole.arc_center.Z)
        for other in body.children:
            if other is plate:
                continue
            with self.subTest(other=other.label):
                overlap = plate & other
                self.assertLess(overlap.volume if overlap is not None else 0, 1e-6)


if __name__ == "__main__":
    unittest.main()
