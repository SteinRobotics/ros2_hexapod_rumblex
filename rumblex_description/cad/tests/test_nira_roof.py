"""Roof geometry, locating frame fit, and separate material exports."""

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import ezdxf
from ezdxf.bbox import extents
from build123d import Face, extrude, import_step

from robot_nira import dxf_directory, torso_common, torso_layer_3, torso_layer_4


class NiraRoofTests(unittest.TestCase):
    def test_original_roof_and_three_mm_frame_seat_without_overlap(self):
        surface = torso_layer_4.build_surface()
        assembly = torso_layer_4.build_model(surface)
        self.assertEqual(len(assembly.children), 2)
        self.assertEqual(len(assembly.solids()), 2)
        roof, frame = assembly.children
        original = extrude(surface, amount=torso_common.THICKNESS)
        self.assertLess((roof - original).volume + (original - roof).volume, 1e-6)
        self.assertAlmostEqual(frame.bounding_box().min.Z, -3)
        self.assertAlmostEqual(frame.bounding_box().max.Z, roof.bounding_box().min.Z)
        self.assertAlmostEqual(frame.distance_to(roof), 0)
        self.assertLess((frame & roof).volume, 1e-6)
        for part in assembly.children:
            self.assertTrue(part.is_valid)
            self.assertEqual(len(part.solids()), 1)

    def test_frame_matches_support_opening_with_five_mm_rim(self):
        support = torso_layer_3.build_surface()
        opening = max((Face(wire) for wire in support.faces()[0].inner_wires()),
                      key=lambda face: face.area)
        frame = torso_layer_4.build_frame_surface().faces()[0]
        self.assertEqual(len(frame.inner_wires()), 1)
        outer = Face(frame.outer_wire())
        self.assertLess((outer - opening).area + (opening - outer).area, 1e-6)
        inner = Face(frame.inner_wires()[0])
        self.assertAlmostEqual(frame.outer_wire().distance_to(frame.inner_wires()[0]), 5)
        outer_bounds, inner_bounds = outer.bounding_box(), inner.bounding_box()
        for axis in ('X', 'Y'):
            self.assertAlmostEqual(getattr(inner_bounds.min, axis) - getattr(outer_bounds.min, axis), 5)
            self.assertAlmostEqual(getattr(outer_bounds.max, axis) - getattr(inner_bounds.max, axis), 5)
        self.assertLess((frame & support).area, 1e-6)

    def test_export_keeps_roof_dxf_and_adds_three_mm_frame(self):
        with tempfile.TemporaryDirectory() as temp:
            destination = Path(temp)
            with patch.object(torso_layer_4, 'EXPORT_DIR', destination), \
                 patch.object(torso_layer_4, 'dxf_directory',
                              side_effect=lambda thickness: dxf_directory(thickness, export_dir=destination)), \
                 patch.object(torso_layer_4, 'show'):
                torso_layer_4.main()
            roof_doc = ezdxf.readfile(destination / 'dxf/dxf_1p5mm/torso_layer_4.dxf')
            engraving = list(roof_doc.modelspace().query('*[layer=="ENGRAVE"]'))
            self.assertTrue(engraving)
            self.assertEqual(roof_doc.layers.get('ENGRAVE').color, 5)
            frame_face = torso_layer_4.build_frame_surface().faces()[0]
            self.assertAlmostEqual(sum(entity.dxf.start.distance(entity.dxf.end)
                                       for entity in engraving), frame_face.outer_wire().length)
            engraving_bounds = extents(engraving)
            expected = frame_face.bounding_box()
            for axis in ('x', 'y'):
                self.assertAlmostEqual(getattr(engraving_bounds.extmin, axis),
                                       getattr(expected.min, axis.upper()))
                self.assertAlmostEqual(getattr(engraving_bounds.extmax, axis),
                                       getattr(expected.max, axis.upper()))
            self.assertTrue(list(roof_doc.modelspace().query('*[layer=="0"]')))
            doc = ezdxf.readfile(destination / 'dxf/dxf_3mm/torso_layer_4_frame.dxf')
            self.assertEqual(doc.units, ezdxf.units.MM)
            bounds = extents(doc.modelspace())
            expected = torso_layer_4.build_frame_surface().bounding_box()
            self.assertAlmostEqual(bounds.extmin.x, expected.min.X)
            self.assertAlmostEqual(bounds.extmax.x, expected.max.X)
            self.assertAlmostEqual(bounds.extmin.y, expected.min.Y)
            self.assertAlmostEqual(bounds.extmax.y, expected.max.Y)
            self.assertEqual(len(import_step(destination / 'step/torso_layer_4.step').solids()), 2)


if __name__ == '__main__':
    unittest.main()
