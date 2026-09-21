"""Check chassis tab engagement and clearance against the assembled body."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import unittest

from build123d import Box, Pos

import robot_nox.assembly_body as assembly_body
import robot_nox.body_common as body_common
import robot_nox.chassis_side as chassis_side


class ChassisSideTests(unittest.TestCase):
    def test_plates_fit_body_slots(self):
        body = assembly_body.build_assembly()
        layers = {p.label: p for p in body.children if p.label.startswith("body_layer_")}
        plates = [p for p in body.children if p.label.startswith("chassis_")]
        self.assertEqual({p.label for p in plates}, {
            "chassis_front", "chassis_back", "chassis_left", "chassis_right",
            "chassis_diagonal_front_left", "chassis_diagonal_front_right",
            "chassis_diagonal_back_left", "chassis_diagonal_back_right",
        })
        for plate in plates:
            with self.subTest(plate=plate.label):
                self.assertTrue(plate.is_valid)
                self.assertEqual(len(plate.solids()), 1)
                bottom_layer = 2 if plate.label.startswith("chassis_diagonal_") else 1
                self.assertAlmostEqual(plate.bounding_box().min.Z,
                                       layers[f"body_layer_{bottom_layer}"].bounding_box().min.Z)
                self.assertAlmostEqual(plate.bounding_box().max.Z,
                                       layers["body_layer_3"].bounding_box().max.Z)
                for other in body.children:
                    if other is plate:
                        continue
                    overlap = plate & other
                    self.assertLess(overlap.volume if overlap is not None else 0, 1e-6,
                                    f"Collision with {other.label}")
                tabs = 6 if plate.label in ("chassis_left", "chassis_right") else 3
                if tabs == 6:
                    # Between the U's legs, only the top rail crosses X=0.
                    rail = plate & (Pos(0, 0, 100) * Box(1, 300, 200))
                    self.assertAlmostEqual(rail.bounding_box().min.Z,
                                           layers["body_layer_2"].bounding_box().max.Z)
                    self.assertAlmostEqual(rail.bounding_box().max.Z,
                                           layers["body_layer_3"].bounding_box().min.Z)
                for index in (bottom_layer, 3):
                    z = layers[f"body_layer_{index}"].bounding_box().center().Z
                    slice_part = plate & (Pos(0, 0, z) * Box(300, 300, body_common.THICKNESS))
                    self.assertEqual(len(slice_part.solids()), tabs)
                    self.assertAlmostEqual(slice_part.volume, tabs * chassis_side.TAB_WIDTH
                                           * chassis_side.THICKNESS * body_common.THICKNESS)


if __name__ == "__main__":
    unittest.main()
