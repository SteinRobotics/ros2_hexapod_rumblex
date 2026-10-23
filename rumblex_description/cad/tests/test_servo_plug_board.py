"""Grid, clearance and mounting checks for the combined servo connector PCB."""

import unittest
from math import ceil

from build123d import Circle, Face, Pos, Rectangle, Rot

from boards import board_servo_plug as board
from robot_nira import board_layout, torso_layer_0
from robot_nira.assembly_torso_with_servos import place_board


class ServoPlugBoardTests(unittest.TestCase):
    def test_pins_and_drilled_spacer_holes_share_grid(self):
        pins = board.pin_positions()
        mounts = board.default_hole_positions(board.cfg)
        self.assertEqual(len(set(pins)), 30)
        self.assertEqual(len(set(mounts)), 4)
        self.assertEqual(board.cfg.hole_diameter, 2.0)
        for x, y in pins + mounts:
            self.assertAlmostEqual(x / 2.54, round(x / 2.54))
            self.assertAlmostEqual(y / 2.54, round(y / 2.54))
        for x, y in mounts:
            self.assertGreater(abs(x), max(abs(px) + board.PLUG_WIDTH / 2
                                           for px, _ in board.PLUG_POSITIONS))
            self.assertGreater(abs(y), max(abs(py) + board.PLUG_LENGTH / 2
                                           for _, py in board.PLUG_POSITIONS))

    def test_ten_plugs_and_four_spacers_fit_board(self):
        model = board.build_board_with_spacers()
        pcb_and_plugs, *spacers = model.children
        pcb, *plugs = pcb_and_plugs.children
        self.assertEqual(len(plugs), 10)
        self.assertEqual(len(spacers), 4)
        self.assertAlmostEqual(pcb.bounding_box().size.Z, 1.6)
        self.assertAlmostEqual(pcb_and_plugs.bounding_box().min.Z, 0)
        self.assertAlmostEqual(pcb_and_plugs.bounding_box().max.Z, board.cfg.thickness)
        self.assertEqual(len(board.build_surface().faces()[0].inner_wires()), 34)
        self.assertTrue(all(solid.is_valid for solid in model.solids()))
        centers = [plug.bounding_box().center() for plug in plugs]
        self.assertEqual(len({round(p.Y, 6) for p in centers}), 2)
        columns = sorted({round(p.X, 6) for p in centers})
        self.assertEqual(len(columns), 5)
        # Adjacent plugs use the smallest grid spacing that avoids overlap.
        for left, right in zip(columns, columns[1:]):
            self.assertAlmostEqual(right - left,
                                   ceil(board.PLUG_WIDTH / 2.54) * 2.54)
        rows = sorted({round(p.Y, 6) for p in centers})
        self.assertAlmostEqual(rows[1] - rows[0], ceil(board.PLUG_LENGTH / 2.54) * 2.54)
        self.assertAlmostEqual(pcb.bounding_box().size.X, 35.56)
        self.assertAlmostEqual(pcb.bounding_box().size.Y, 25.4)
        actual_pins = set()
        for plug in plugs:
            bounds = plug.bounding_box()
            self.assertAlmostEqual(bounds.size.X, board.PLUG_WIDTH)
            self.assertAlmostEqual(bounds.size.Y, board.PLUG_LENGTH)
            contacts = [(plug.location * contact).bounding_box().center()
                        for contact in plug.children[1:]]
            self.assertEqual(len({round(p.X, 6) for p in contacts}), 1)
            for contact in contacts:
                actual_pins.add((round(contact.X, 6), round(contact.Y, 6)))
        self.assertEqual(actual_pins,
                         {(round(x, 6), round(y, 6)) for x, y in board.pin_positions()})
        for i, plug in enumerate(plugs):
            for other in [*plugs[i + 1:], *spacers]:
                self.assertGreater(plug.distance_to(other), 0.1)

    def test_single_board_clears_neighbors_and_spacers_match_plate(self):
        mounts = [m for m in board_layout.BOARD_MOUNTS if m.module is board]
        self.assertEqual(len(mounts), 1)
        mount = mounts[0]
        footprint = Pos(mount.placement.x, mount.placement.y) * Rectangle(
            board.cfg.length, board.cfg.width)
        for other in board_layout.BOARD_MOUNTS:
            if other is mount:
                continue
            neighbor = (Pos(other.placement.x, other.placement.y)
                        * Rot(Z=other.placement.rotation)
                        * Rectangle(other.module.cfg.length, other.module.cfg.width))
            self.assertGreater(footprint.distance_to(neighbor), 0.5, other.name)

        surface = torso_layer_0.build_surface()
        holes = [Face(wire) for wire in surface.faces()[0].inner_wires()]
        placed = place_board(board, mount.placement)
        for post in placed.children[1:]:
            actual = placed.location * post
            center = actual.bounding_box().center()
            self.assertAlmostEqual(actual.bounding_box().min.Z, torso_layer_0.torso_common.THICKNESS)
            expected_hole = Pos(center.X, center.Y) * Circle(1.0)
            self.assertTrue(any((hole.center() - expected_hole.center()).length < 1e-6
                                and abs(hole.area - expected_hole.area) < 1e-6
                                for hole in holes))

    def test_nox_base_uses_combined_mounting_pattern(self):
        from robot_nox import torso_layer_0 as nox_base

        placement = nox_base.LOCATION_SERVO_PLUG
        surface = nox_base.build_surface()
        holes = [Face(wire) for wire in surface.faces()[0].inner_wires()]
        for x, y in board.default_hole_positions(board.cfg):
            center = Pos(placement.x + x, placement.y + y) * Circle(1.0)
            self.assertTrue(any((hole.center() - center.center()).length < 1e-6
                                and abs(hole.area - center.area) < 1e-6
                                for hole in holes))


if __name__ == "__main__":
    unittest.main()
