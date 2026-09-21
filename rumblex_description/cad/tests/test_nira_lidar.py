"""Check Nira's sensor support, plate fit and the complete robot's scan window."""

import math
import unittest

from build123d import (
    Box, BuildPart, BuildSketch, Circle, Cylinder, Locations, Mode, Part, Plane, Polygon, Pos,
    add, extrude,
)

from robot_nira import body_common, lidar_ydlidar_tmini, lidar_interface_housing
from robot_nira.assembly_complete import build_assembly
from robot_nira.lidar_layout import LIDAR_X, LIDAR_Y, canopy_surface


def leaves(part):
    if part.children:
        for child in part.children:
            yield from leaves(child)
    else:
        yield part


class NiraLidarTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = build_assembly()
        cls.body = cls.robot.children[0].children[0]
        cls.parts = {part.label: part for part in cls.body.children}
        cls.lidar = cls.parts['lidar_ydlidar_tmini']

    def test_sensor_stack_clears_interface_board_in_front_third(self):
        deck = self.parts['body_layer_2']
        interface = self.parts['board_ydlidar_tmini_interface']
        housing = self.parts['lidar_interface_housing']
        box = self.lidar.bounding_box()
        self.assertAlmostEqual(interface.bounding_box().min.Z, deck.bounding_box().max.Z)
        self.assertLess(interface.distance_to(deck), 1e-6)
        self.assertAlmostEqual(housing.bounding_box().min.Z, deck.bounding_box().max.Z)
        self.assertAlmostEqual(box.min.Z, housing.bounding_box().max.Z)
        self.assertLess(self.lidar.distance_to(housing), 1e-6)
        self.assertGreaterEqual(box.min.Z - lidar_interface_housing.WALL
                                - interface.bounding_box().max.Z, 1.0 - 1e-6)
        self.assertAlmostEqual(interface.joints['mount'].location.position.X, box.center().X)
        self.assertAlmostEqual(interface.joints['mount'].location.position.Y, box.center().Y)
        self.assertGreater(box.min.X, body_common.rect_w / 6)
        self.assertAlmostEqual(box.center().X, LIDAR_X)
        self.assertAlmostEqual(box.center().Y, LIDAR_Y)

    def test_canopy_tips_end_over_scanner_with_vertical_clearance(self):
        for name in ('body_layer_3', 'body_layer_4'):
            with self.subTest(plate=name):
                plate = self.parts[name]
                bounds = plate.bounding_box()
                self.assertAlmostEqual(bounds.max.X, self.lidar.bounding_box().center().X)
                self.assertGreaterEqual(bounds.min.Z - self.lidar.bounding_box().max.Z, 3.0)
                # Both layers must contain the nose, including the lower frame
                # whose old service opening used to remove the entire tip.
                tip = Pos(LIDAR_X - 1, LIDAR_Y, bounds.center().Z) * Box(1, 1, 1)
                self.assertAlmostEqual((tip & plate).volume, tip.volume)

    def test_housing_mounts_and_connector_remain_accessible(self):
        housing = self.parts['lidar_interface_housing']
        deck = self.parts['body_layer_2']
        interface = self.parts['board_ydlidar_tmini_interface']
        z = deck.bounding_box().max.Z
        for x, y in lidar_interface_housing.MOUNT_POINTS:
            bore = Pos(LIDAR_X + x, LIDAR_Y + y, z) * Cylinder(1.5, 10)
            for part in (housing, deck):
                overlap = bore & part
                self.assertLess(overlap.volume if overlap else 0, 1e-6)
        connector = interface.joints['connector'].location.position
        access = Pos(connector.X + 5, connector.Y, connector.Z) * Box(10, 10, 4)
        overlap = housing & access
        self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_modified_plates_are_valid_and_fit_the_body(self):
        for part in self.body.children:
            with self.subTest(part=part.label):
                self.assertTrue(part.is_valid)
                for component in leaves(part):
                    self.assertEqual(len(component.solids()), 1, component.label)
            if not part.label.startswith(('chassis_', 'lidar_')):
                continue
            for other in self.body.children:
                if other is part:
                    continue
                with self.subTest(part=part.label, other=other.label):
                    overlap = part & other
                    self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_retained_upper_tabs_engage_layer_3(self):
        top = self.parts['body_layer_3'].bounding_box()
        tab_slice = Pos(0, 0, top.center().Z) * Box(300, 200, body_common.THICKNESS)
        for part in self.body.children:
            if not part.label.startswith('chassis_'):
                continue
            with self.subTest(part=part.label):
                tabs = part & tab_slice
                if 'front' in part.label:
                    self.assertLess(tabs.volume if tabs else 0, 1e-6)
                else:
                    self.assertEqual(len(tabs.solids()), 3)
                    self.assertAlmostEqual(tabs.volume, 3 * 8 * 1.5 * 1.5)
                    # The tab footprints sit inside the canopy perimeter.
                    with BuildPart() as outline:
                        with BuildSketch(Plane.XY.offset(top.min.Z)):
                            add(canopy_surface())
                        extrude(amount=body_common.THICKNESS)
                    self.assertAlmostEqual((tabs & outline.part).volume, tabs.volume)

    def test_complete_robot_clears_270_degree_scan_band(self):
        # Continuous swept volume, including both +/-135-degree boundaries,
        # checked against head, legs, electronics, armor and all body spacers.
        z = self.lidar.bounding_box().min.Z + lidar_ydlidar_tmini.SCAN_HEIGHT
        with BuildPart() as sector:
            with BuildSketch(Plane.XY.offset(z - 2)):
                Polygon((LIDAR_X, LIDAR_Y), *[
                    (LIDAR_X + 600 * math.cos(math.radians(angle)),
                     LIDAR_Y + 600 * math.sin(math.radians(angle)))
                    for angle in range(-135, 136, 3)
                ], align=None)
                with Locations((LIDAR_X, LIDAR_Y)):
                    Circle(20, mode=Mode.SUBTRACT)
            extrude(amount=4)
        for part in leaves(self.robot):
            if part is self.lidar:
                continue
            # Child geometry is local to its parent; include the head/leg pose
            # and every enclosing assembly before testing the world-space band.
            # Wrap only the geometry so located() doesn't copy the assembly tree.
            part = Part(part.wrapped, label=part.label).located(part.global_location)
            bounds = part.bounding_box()
            if bounds.max.Z < z - 2 or bounds.min.Z > z + 2:
                continue
            with self.subTest(part=part.label):
                overlap = part & sector.part
                self.assertLess(overlap.volume if overlap else 0, 1e-6)


if __name__ == '__main__':
    unittest.main()
