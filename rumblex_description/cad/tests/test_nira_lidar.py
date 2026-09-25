"""Check Nira's sensor support, plate fit and the complete robot's scan window."""

import math
import unittest

from build123d import (
    Box, BuildPart, BuildSketch, Circle, Cylinder, Locations, Mode, Part, Plane, Polygon, Pos,
    add, extrude,
)

from robot_nira import body_common, chassis_side, lidar_ydlidar_tmini, lidar_interface_housing
from robot_nira.assembly_complete import build_assembly
from robot_nira.lidar_layout import LIDAR_ASSEMBLY_X, LIDAR_X, LIDAR_Y, canopy_surface


def leaves(part):
    if part.children:
        for child in part.children:
            yield from leaves(child)
    else:
        yield part


class NiraLidarHousingCutTests(unittest.TestCase):
    def test_housing_has_five_flat_interlocking_parts(self):
        panels = lidar_interface_housing.flat_parts()
        self.assertEqual(len(panels), 5)
        for name, panel in panels.items():
            with self.subTest(panel=name):
                self.assertTrue(panel.is_valid)
                self.assertEqual(len(panel.solids()), 1)
                self.assertAlmostEqual(panel.bounding_box().size.Z,
                                       lidar_interface_housing.WALL)
        assembled = lidar_interface_housing.assemble(panels)
        self.assertTrue(assembled.is_valid)
        self.assertEqual(len(assembled.solids()), 5)
        self.assertAlmostEqual(assembled.volume,
                               sum(panel.volume for panel in panels.values()))


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
        self.assertAlmostEqual(housing.bounding_box().min.Z, deck.bounding_box().min.Z)
        self.assertAlmostEqual(box.min.Z, housing.bounding_box().max.Z)
        self.assertLess(self.lidar.distance_to(housing), 1e-6)
        self.assertGreaterEqual(box.min.Z - lidar_interface_housing.WALL
                                - interface.bounding_box().max.Z, 1.0 - 1e-6)
        self.assertAlmostEqual(interface.joints['mount'].location.position.X, box.center().X)
        self.assertAlmostEqual(interface.joints['mount'].location.position.Y, box.center().Y)
        self.assertGreater(box.min.X, body_common.rect_w / 6)
        self.assertAlmostEqual(box.center().X, LIDAR_ASSEMBLY_X)
        self.assertAlmostEqual(box.center().Y, LIDAR_Y)

    def test_canopy_rear_frame_and_roof_clear_scanner(self):
        rear_frame = self.parts['body_layer_3']
        self.assertAlmostEqual(rear_frame.bounding_box().max.X, 0)
        for name in ('body_layer_3', 'body_layer_4'):
            with self.subTest(plate=name):
                plate = self.parts[name]
                bounds = plate.bounding_box()
                self.assertGreaterEqual(bounds.min.Z - self.lidar.bounding_box().max.Z, 3.0)
        roof = self.parts['body_layer_4']
        self.assertAlmostEqual(roof.bounding_box().max.X, LIDAR_X)
        tip = Pos(LIDAR_X - 1, LIDAR_Y, roof.bounding_box().center().Z) * Box(1, 1, 1)
        self.assertAlmostEqual((tip & roof).volume, tip.volume)

    def test_rear_frame_retains_joint_slots_and_spacer_holes(self):
        frame = self.parts['body_layer_3']
        z = frame.bounding_box().center().Z
        for location in [*body_common.rectangle_slots_locations,
                         *body_common.diagonal_slots_locations]:
            if location.position.X < 0:
                with self.subTest(slot=location):
                    probe = Pos(location.position.X, location.position.Y, z) * Box(1, 1, 1)
                    self.assertLess((probe & frame).volume, 1e-6)
        for location in body_common.hole_locations:
            if location.position.X < 0:
                with self.subTest(hole=location):
                    bore = Pos(location.position.X, location.position.Y, z) * Cylinder(
                        body_common.hole_radius / 2, body_common.THICKNESS
                    )
                    self.assertLess((bore & frame).volume, 1e-6)
                    rim = Pos(location.position.X + body_common.hole_radius + 1,
                              location.position.Y, z) * Box(1, 1, 1)
                    self.assertGreater((rim & frame).volume, 0)

    def test_housing_fingers_fit_deck_slots_and_connector_remains_accessible(self):
        housing = self.parts['lidar_interface_housing']
        deck = self.parts['body_layer_2']
        interface = self.parts['board_ydlidar_tmini_interface']
        z = deck.bounding_box().center().Z
        for x in lidar_interface_housing.DECK_TAB_X:
            for y in (-lidar_interface_housing.DECK_SLOT_Y,
                      lidar_interface_housing.DECK_SLOT_Y):
                with self.subTest(x=x, y=y):
                    finger = Pos(LIDAR_ASSEMBLY_X + x, y, z) * Box(
                        lidar_interface_housing.DECK_TAB_WIDTH,
                        lidar_interface_housing.WALL,
                        body_common.THICKNESS,
                    )
                    self.assertAlmostEqual((housing & finger).volume, finger.volume)
                    self.assertLess((deck & finger).volume, 1e-6)
        connector = interface.joints['connector'].location.position
        access = Pos(connector.X + 5, connector.Y, connector.Z) * Box(10, 10, 4)
        overlap = housing & access
        self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_modified_plates_are_valid_and_fit_the_body(self):
        for part in self.body.children:
            with self.subTest(part=part.label):
                self.assertTrue(part.is_valid)
                for component in leaves(part):
                    expected = 5 if component.label == 'lidar_interface_housing' else 1
                    self.assertEqual(len(component.solids()), expected, component.label)
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
            if not part.label.startswith('chassis_') or part.label == 'chassis_slope_cover':
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

    def test_front_plates_use_three_individual_deck_fingers(self):
        deck = self.parts['body_layer_2']
        deck_box = deck.bounding_box()
        z = deck_box.center().Z
        finger_volume = (chassis_side.TAB_WIDTH * chassis_side.THICKNESS
                         * body_common.THICKNESS)

        front = self.parts['chassis_front']
        self.assertAlmostEqual(front.bounding_box().max.Z, deck_box.max.Z)
        for name in ('chassis_front', 'chassis_diagonal_front_left',
                     'chassis_diagonal_front_right'):
            with self.subTest(plate=name):
                fingers = self.parts[name] & Pos(0, 0, z) * Box(300, 200, body_common.THICKNESS)
                self.assertEqual(len(fingers.solids()), 3)
                self.assertAlmostEqual(fingers.volume, 3 * finger_volume)
                self.assertLess((fingers & deck).volume, 1e-6)

        for y in (-chassis_side.TAB_PITCH, 0, chassis_side.TAB_PITCH):
            slot = Pos(100, y, z) * Box(body_common.rectangle_slots_height,
                                         chassis_side.TAB_WIDTH, body_common.THICKNESS)
            self.assertLess((slot & deck).volume, 1e-6)
        for y in (-chassis_side.TAB_PITCH / 2, chassis_side.TAB_PITCH / 2):
            web = Pos(100, y, z) * Box(body_common.rectangle_slots_height,
                                        body_common.rectangle_slots_height,
                                        body_common.THICKNESS)
            self.assertAlmostEqual((web & deck).volume, web.volume)

    def test_slope_cover_closes_gap_between_side_plates(self):
        cover = self.parts['chassis_slope_cover']
        left = self.parts['chassis_left']
        right = self.parts['chassis_right']
        top = self.parts['body_layer_3'].bounding_box().min.Z

        self.assertTrue(cover.is_valid)
        self.assertEqual(len(cover.solids()), 1)
        self.assertAlmostEqual(cover.bounding_box().max.X, chassis_side.SLOPE_END_X)
        self.assertAlmostEqual(cover.bounding_box().max.Z, top)
        for side in (left, right):
            self.assertLess(cover.distance_to(side), 1e-6)
            overlap = cover & side
            self.assertLess(overlap.volume if overlap else 0, 1e-6)
            # Five cover fingers occupy matching notches across the complete
            # side-plate thickness; the intervening edge stays against it.
            side_band = Pos(0, side.bounding_box().center().Y, top - 20) * Box(
                300, chassis_side.THICKNESS, 100)
            fingers = cover & side_band
            self.assertEqual(len(fingers.solids()), chassis_side.JOINT_FINGER_COUNT)
            self.assertAlmostEqual(
                fingers.volume,
                chassis_side.JOINT_FINGER_COUNT
                * chassis_side.JOINT_FINGER_LENGTH
                * chassis_side.THICKNESS ** 2,
            )
        # The cover spans the centre at mid-slope while clearing the housing
        # at its lower edge.
        midpoint = (chassis_side.SHOULDER_TOP_X + chassis_side.SLOPE_END_X) / 2
        probe = Pos(midpoint, 0, top - 21 - 0.75) * Box(1, 1, 1)
        self.assertGreater((cover & probe).volume, 0)
        housing_overlap = cover & self.parts['lidar_interface_housing']
        self.assertLess(housing_overlap.volume if housing_overlap else 0, 1e-6)

    def test_two_speakers_face_perforations_from_inside(self):
        cover = self.parts['chassis_slope_cover']
        holes = chassis_side.speaker_hole_positions()
        self.assertGreater(len(holes), 20)
        cylindrical_faces = [face for face in cover.faces()
                             if face.geom_type.name == 'CYLINDER']
        self.assertEqual(len(cylindrical_faces), 2 * len(holes))

        for name, y in zip(('left', 'right'), chassis_side.SPEAKER_Y_POSITIONS):
            speaker = self.parts[f'loudspeaker_{name}']
            with self.subTest(speaker=name):
                self.assertTrue(speaker.is_valid)
                self.assertEqual(len(speaker.solids()), 1)
                self.assertAlmostEqual(speaker.bounding_box().center().Y, y)
                self.assertLess(speaker.distance_to(cover), 1e-6)
                overlap = speaker & cover
                self.assertLess(overlap.volume if overlap else 0, 1e-6)
                plane = chassis_side.speaker_plane(
                    self.parts['body_layer_3'].bounding_box().min.Z, y)
                for across, along in (holes[0], (0, 0), holes[-1]):
                    probe = (plane.location
                             * Pos(across, along, -chassis_side.THICKNESS / 2)
                             * Cylinder(chassis_side.SPEAKER_HOLE_RADIUS / 2,
                                        chassis_side.THICKNESS + 0.2))
                    self.assertFalse(cover & probe)

    def test_complete_robot_clears_270_degree_scan_band(self):
        # Continuous swept volume, including both +/-135-degree boundaries,
        # checked against head, legs, electronics, armor and all body spacers.
        z = self.lidar.bounding_box().min.Z + lidar_ydlidar_tmini.SCAN_HEIGHT
        with BuildPart() as sector:
            with BuildSketch(Plane.XY.offset(z - 2)):
                Polygon((LIDAR_ASSEMBLY_X, LIDAR_Y), *[
                    (LIDAR_ASSEMBLY_X + 600 * math.cos(math.radians(angle)),
                     LIDAR_Y + 600 * math.sin(math.radians(angle)))
                    for angle in range(-135, 136, 3)
                ], align=None)
                with Locations((LIDAR_ASSEMBLY_X, LIDAR_Y)):
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
