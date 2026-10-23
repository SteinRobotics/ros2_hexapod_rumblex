"""Board engraving stays on the base plate and separate from cutting paths."""

from pathlib import Path
import tempfile
import unittest

import ezdxf
from build123d import ExportDXF

from robot_nira import torso_layer_0
from robot_nira.board_layout import BOARD_MOUNTS


class BoardLabelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.surface = torso_layer_0.build_surface()
        cls.labels = torso_layer_0.build_board_labels()

    def test_labels_fit_on_plate_without_crossing_holes(self):
        self.assertEqual(len(self.labels), len(BOARD_MOUNTS))
        self.assertEqual(len({mount.name for mount in BOARD_MOUNTS}), len(BOARD_MOUNTS))
        for mount, label in zip(BOARD_MOUNTS, self.labels, strict=True):
            with self.subTest(board=mount.name):
                self.assertGreater(label.area, 0)
                self.assertLess((label - self.surface).area, 1e-6)

    def test_dxf_has_blue_engraving_and_preserves_cut_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            baseline = Path(folder) / "cuts.dxf"
            labelled = Path(folder) / "labelled.dxf"
            drawing = ExportDXF()
            drawing.add_shape(self.surface)
            drawing.write(str(baseline))
            torso_layer_0.build_dxf(self.surface).write(str(labelled))
            original = ezdxf.readfile(baseline)
            result = ezdxf.readfile(labelled)

            self.assertFalse(result.audit().has_errors)
            self.assertEqual(result.layers.get("ENGRAVE").dxf.color, 5)
            cuts = [e for e in result.modelspace() if e.dxf.layer != "ENGRAVE"]
            self.assertEqual(len(cuts), len(original.modelspace()))
            self.assertEqual([e.dxftype() for e in cuts],
                             [e.dxftype() for e in original.modelspace()])
            engraving = [e for e in result.modelspace() if e.dxf.layer == "ENGRAVE"]
            self.assertGreater(len(engraving), 0)
            self.assertTrue(all(e.dxf.color in (5, 256) for e in engraving))


if __name__ == "__main__":
    unittest.main()
