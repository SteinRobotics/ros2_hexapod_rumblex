"""Manufacturing invariants, independent of the large robot assemblies."""

from dataclasses import replace
import json
import tempfile
import unittest

from build123d import Circle, Rectangle, Pos
import ezdxf
from shapely.affinity import translate
from laser_cutting.geometry import make_profile
from laser_cutting.nesting import Settings, nest, validate
from laser_cutting.output import write_job
from laser_cutting.robots import Component, robot_components


class LaserCuttingTests(unittest.TestCase):
    def profile(self, name="plate", quantity=1, shape=None, thickness=1.5, kerf=0):
        if shape is None:
            shape = Rectangle(20, 10) - Circle(2)
        return make_profile(Component(name, quantity, thickness, shape), 0.005, kerf)

    def test_quantity_holes_units_and_roundtrip(self):
        p = self.profile(quantity=6)
        settings = Settings(width=80, height=60, rotation_step=90, trials=2)
        sheets = nest([p], settings)
        self.assertEqual(sum(map(len, sheets)), 6)
        with tempfile.TemporaryDirectory() as temp:
            destination, report = write_job("test", [p], {1.5: sheets}, settings, temp)
            self.assertEqual(report["placed_components"], 6)
            self.assertEqual(json.loads((destination / "summary.json").read_text())["robot"], "test")
            for sheet in report["sheets"]:
                doc = ezdxf.readfile(destination / sheet["file"])
                self.assertEqual(doc.units, ezdxf.units.MM)
                self.assertEqual(len(doc.modelspace().query("CIRCLE")), sheet["placed_components"])
            with self.assertRaisesRegex(ValueError, "already exists"):
                write_job("test", [p], {1.5: sheets}, settings, temp)

    def test_actual_geometry_nests_inside_hole(self):
        ring = self.profile("ring", shape=Rectangle(70, 70) - Rectangle(50, 50))
        insert = self.profile("insert", shape=Rectangle(35, 35))
        sheets = nest([ring, insert], Settings(width=80, height=80, rotation_step=90, trials=1))
        self.assertEqual(len(sheets), 1)
        a, b = sheets[0]
        self.assertGreater(a.polygon.envelope.intersection(b.polygon.envelope).area, 1000)
        self.assertEqual(a.polygon.intersection(b.polygon).area, 0)

    def test_rotation_and_multiple_sheets(self):
        p = self.profile(quantity=3, shape=Rectangle(70, 25))
        settings = Settings(width=40, height=80, rotation_step=90, trials=1)
        sheets = nest([p], settings)
        self.assertEqual(len(sheets), 3)
        self.assertTrue(all(part.angle in (90, 270) for sheet in sheets for part in sheet))

    def test_missing_duplicate_overlap_and_boundary(self):
        p = self.profile(quantity=2)
        s = Settings(rotation_step=90, trials=1)
        sheets = nest([p], s)
        with self.assertRaisesRegex(ValueError, "BOM mismatch"):
            validate([p], [sheets[0][:-1]], s)
        with self.assertRaisesRegex(ValueError, "BOM mismatch"):
            validate([p], [sheets[0] + [sheets[0][0]]], s)
        a, b = sheets[0]
        with self.assertRaisesRegex(ValueError, "Overlap"):
            validate([p], [[a, replace(b, polygon=a.polygon)]], s)
        with self.assertRaisesRegex(ValueError, "boundary"):
            validate([p], [[a, replace(b, polygon=translate(b.polygon, -500, 0))]], s)

    def test_bad_bom_geometry_and_parameters(self):
        with self.assertRaisesRegex(ValueError, "cannot fit"):
            nest([self.profile(shape=Rectangle(500, 500))], Settings(rotation_step=90))
        with self.assertRaisesRegex(ValueError, "thickness"):
            self.profile(thickness=2)
        with self.assertRaisesRegex(ValueError, "quantity"):
            self.profile(quantity=0)
        with self.assertRaisesRegex(ValueError, "connected physical"):
            self.profile(shape=Rectangle(10, 10) + Pos(30, 0)*Rectangle(10, 10))
        for s in (Settings(clearance=-1), Settings(kerf=float("nan")), Settings(margin=151),
                  Settings(resolution=0), Settings(trials=0)):
            with self.assertRaises(ValueError):
                s.validate()
        with self.assertRaisesRegex(ValueError, "Unknown robot"):
            robot_components("undefined")

    def test_kerf_direction_and_feature_loss(self):
        nominal = self.profile()
        compensated = self.profile(kerf=0.2)
        self.assertAlmostEqual(compensated.polygon.bounds[0], nominal.polygon.bounds[0] - 0.1)
        self.assertLess(compensated.polygon.interiors[0].length, nominal.polygon.interiors[0].length)
        self.assertEqual(len(compensated.polygon.interiors), 1)
        with self.assertRaisesRegex(ValueError, "topology"):
            self.profile(kerf=5)
        settings = Settings(kerf=0.2, rotation_step=90, trials=1)
        sheets = nest([compensated], settings)
        with tempfile.TemporaryDirectory() as temp:
            destination, _ = write_job("kerf", [compensated], {1.5: sheets}, settings, temp)
            doc = ezdxf.readfile(destination / "1.5mm/sheet_01.dxf")
            self.assertEqual(len(doc.modelspace().query("CIRCLE")), 1)
            self.assertAlmostEqual(doc.modelspace().query("CIRCLE")[0].dxf.radius, 1.9)

    def test_mixed_thickness_rejected(self):
        a = self.profile("thin", thickness=1)
        b = self.profile("thick", thickness=3)
        s = Settings(rotation_step=90, trials=1)
        with self.assertRaisesRegex(ValueError, "Mixed material"):
            nest([a, b], s)

    def test_nira_design_bom_and_all_cutting_faces(self):
        components = robot_components("nira")
        self.assertEqual(sum(c.quantity for c in components), 48)
        self.assertEqual({t: sum(c.quantity for c in components if c.thickness == t)
                          for t in (1, 1.5, 3)}, {1: 1, 1.5: 38, 3: 9})
        by_name = {c.name: c for c in components}
        for name in ("foot_front", "foot_back", "foot_outer", "foot_connection"):
            self.assertEqual(by_name[name].quantity, 6)
        self.assertEqual(by_name["chassis_diagonal_back_left"].quantity, 1)
        self.assertEqual(by_name["chassis_slope_cover"].quantity, 1)
        self.assertEqual(sum(c.quantity for c in components if c.name.startswith("lidar_housing_")), 5)
        for component in components:
            with self.subTest(component=component.name):
                profile = make_profile(component)
                self.assertEqual(len(profile.polygon.interiors),
                                 len(component.surface.faces()[0].inner_wires()))

    def test_nox_ambiguous_bottom_plate_is_rejected(self):
        components = robot_components("nox")
        base = next(c for c in components if c.name == "torso_layer_0")
        with self.assertRaisesRegex(ValueError, "got 3 faces"):
            make_profile(base)


if __name__ == "__main__":
    unittest.main()
