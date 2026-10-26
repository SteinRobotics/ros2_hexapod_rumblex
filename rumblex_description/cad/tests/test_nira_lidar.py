"""Check Nira's sensor support, plate fit and the complete robot's scan window."""

import math
import unittest

from build123d import (
    Box, BuildPart, BuildSketch, Circle, Cylinder, Face, Locations, Mode, Part, Plane, Polygon, Pos,
    Rot, insert, extrude,
)

from robot_nira import (torso_common, chassis_common,
                        chassis_slope_cover, lidar_ydlidar_tmini, lidar_interface_housing, torso_layer_3)
from robot_nira.assembly_complete import build_assembly
from robot_nira.lidar_layout import (
    CANOPY_HALF_WIDTH, CANOPY_SHOULDER_X, LIDAR_ASSEMBLY_X, LIDAR_X, LIDAR_Y, canopy_surface,
)


def leaves(part):
    if part.children:
        for child in part.children:
            yield from leaves(child)
    else:
        yield part


class NiraCanopyFrameTests(unittest.TestCase):
    def test_closed_frame_has_five_mm_front_rail_and_original_outer_dimensions(self):
        surface = torso_layer_3.build_surface()
        self.assertEqual(len(surface.faces()), 1)
        face = surface.faces()[0]
        opening = max((Face(wire) for wire in face.inner_wires()), key=lambda hole: hole.area)
        self.assertGreater(opening.area, 1000)
        self.assertAlmostEqual(opening.bounding_box().max.X, CANOPY_SHOULDER_X - 5.0)
        bounds = surface.bounding_box()
        self.assertAlmostEqual(bounds.min.X, -torso_common.rect_w / 2)
        self.assertAlmostEqual(bounds.max.X, CANOPY_SHOULDER_X)
        self.assertAlmostEqual(bounds.min.Y, -CANOPY_HALF_WIDTH)
        self.assertAlmostEqual(bounds.max.Y, CANOPY_HALF_WIDTH)
        rail = Pos(CANOPY_SHOULDER_X - 2.5, 0, torso_common.THICKNESS / 2) * Box(
            5, 2 * CANOPY_HALF_WIDTH, torso_common.THICKNESS
        )
        model = torso_layer_3.build_model(surface)
        self.assertTrue(model.is_valid)
        self.assertEqual(len(model.solids()), 1)
        self.assertAlmostEqual((rail & model).volume, rail.volume)


class NiraSlopeCoverTests(unittest.TestCase):
    def test_extended_cover_seats_on_deck_with_matching_enclosed_side_joints(self):
        from robot_nira import chassis_side, torso_layer_2
        from robot_nira.torso_layout import LAYER_1_BOTTOM, LAYER_2_BOTTOM, LAYER_2_TOP, LAYER_3_BOTTOM

        cover = chassis_slope_cover.build_slope_cover(LAYER_3_BOTTOM)
        deck = Pos(Z=LAYER_2_BOTTOM) * torso_layer_2.build_model(torso_layer_2.build_surface())
        self.assertTrue(cover.is_valid)
        self.assertEqual(len(cover.solids()), 1)
        self.assertAlmostEqual(cover.bounding_box().min.Z, LAYER_2_TOP)
        self.assertLess(cover.distance_to(deck), 1e-6)
        overlap = cover & deck
        self.assertLess(overlap.volume if overlap else 0, 1e-6)
        for side in chassis_side.build_plates(LAYER_1_BOTTOM, LAYER_3_BOTTOM):
            if side.label not in ('chassis_left', 'chassis_right'):
                continue
            with self.subTest(side=side.label):
                self.assertTrue(side.is_valid)
                self.assertEqual(len(side.solids()), 1)
                self.assertLess(cover.distance_to(side), 1e-6)
                overlap = cover & side
                self.assertLess(overlap.volume if overlap else 0, 1e-6)
                band = Pos(0, side.bounding_box().center().Y, LAYER_3_BOTTOM) * Box(
                    300, chassis_common.THICKNESS, 200)
                fingers = cover & band
                self.assertEqual(len(fingers.solids()), chassis_common.JOINT_FINGER_COUNT)
                self.assertAlmostEqual(fingers.volume, chassis_common.JOINT_FINGER_COUNT
                                       * chassis_common.JOINT_FINGER_LENGTH
                                       * chassis_common.THICKNESS ** 2)
                holes = max(side.faces(), key=lambda face: face.area).inner_wires()
                for finger in fingers.solids():
                    self.assertTrue(any(
                        (wire.bounding_box().center() - finger.center()).length
                        < chassis_common.THICKNESS for wire in holes))
                overlap = side & deck
                self.assertLess(overlap.volume if overlap else 0, 1e-6)


class NiraLidarHousingCutTests(unittest.TestCase):
    def test_front_is_solid_and_fingers_fill_enclosed_side_slots(self):
        h = lidar_interface_housing
        panel = h.end_wall()
        core = Pos(0, (h.HEIGHT - h.WALL) / 2, h.WALL / 2) * Box(
            2 * (h.HALF_Y - h.WALL), h.HEIGHT - h.WALL, h.WALL)
        self.assertAlmostEqual((panel & core).volume, core.volume)
        self.assertEqual(len(max(panel.faces(), key=lambda f: f.area).inner_wires()), 0)
        assembled = h.build_model()
        roof, negative, positive, front = [Part(p.wrapped) for p in assembled.children[:4]]
        for name, side in (('side_negative_y', negative), ('side_positive_y', positive)):
            with self.subTest(side=name):
                face = max(side.faces(), key=lambda f: f.area)
                slots = face.inner_wires()
                self.assertEqual(len(slots), len(h.CORNER_FINGER_Z))
                for slot in slots:
                    self.assertGreater(slot.distance_to(face.outer_wire()), 1.0)
                y = side.bounding_box().center().Y
                band = Pos(h.MIN_X + h.WALL / 2, y, (h.HEIGHT - h.WALL) / 2) * Box(
                    h.WALL, h.WALL, h.HEIGHT - h.WALL)
                fingers = front & band
                self.assertEqual(len(fingers.solids()), len(h.CORNER_FINGER_Z))
                self.assertAlmostEqual(fingers.volume,
                                       len(h.CORNER_FINGER_Z) * h.FINGER_HEIGHT * h.WALL**2)
                self.assertAlmostEqual(((front + side) & band).volume, band.volume)
                overlap = front & side
                self.assertLess(overlap.volume if overlap else 0, 1e-6)
                # The former upper corner cutbacks now meet the roof with
                # continuous side material, including ahead of the front panel.
                for x in (h.SIDE_MIN_X + 0.5, h.MIN_X + h.WALL / 2, h.MIN_X + 3):
                    below = Pos(x, y, h.HEIGHT - h.WALL - 0.25) * Box(0.5, h.WALL, 0.5)
                    above = Pos(Z=0.5) * below
                    self.assertAlmostEqual(((front + side) & below).volume, below.volume)
                    self.assertAlmostEqual((roof & above).volume, above.volume)

    def test_lidar_position_is_retained_and_mounts_pass_through_extended_lid(self):
        h = lidar_interface_housing
        lid = h.lid()
        sensor = (Pos(h.CENTER_X, 0, h.HEIGHT)
                  * Rot(Z=90) * lidar_ydlidar_tmini.build_model())
        self.assertAlmostEqual(sensor.bounding_box().center().X, -2.0)
        self.assertAlmostEqual(sensor.bounding_box().center().Y,
                               lid.bounding_box().center().Y)
        for index in (1, 2):
            p = sensor.joints[f'mount_{index}'].location.position
            bore = Pos(p.X, p.Y, h.WALL / 2) * Cylinder(
                h.LIDAR_MOUNT_DIAMETER / 2, h.WALL)
            overlap = lid & bore
            self.assertLess(overlap.volume if overlap else 0, 1e-6)
            rim = Pos(p.X + h.LIDAR_MOUNT_DIAMETER / 2 + 0.5,
                      p.Y, h.WALL / 2) * Cylinder(0.2, h.WALL)
            self.assertAlmostEqual((lid & rim).volume, rim.volume)

    def test_open_rear_tunnel_reaches_slope_and_clears_usb_cables(self):
        from robot_nira.torso_layout import LAYER_2_TOP, LAYER_3_BOTTOM

        h = lidar_interface_housing
        housing = (Pos(LIDAR_ASSEMBLY_X, LIDAR_Y, LAYER_2_TOP) * Rot(Z=180)
                   * Part(h.build_model().wrapped))
        cover = chassis_slope_cover.build_slope_cover(LAYER_3_BOTTOM)
        self.assertTrue(cover.is_valid)
        self.assertEqual(len(cover.solids()), 1)
        self.assertGreater(h.MAX_X, h.HALF_X)
        self.assertLess(housing.distance_to(cover), 1e-6)
        overlap = housing & cover
        self.assertLess(overlap.volume if overlap else 0, 1e-6)
        for speaker in chassis_slope_cover.build_speakers(LAYER_3_BOTTOM):
            overlap = housing & speaker
            self.assertLess(overlap.volume if overlap else 0, 1e-6)
        # The whole empty cross-section crosses the former rear panel and the
        # slope, not just narrow holes at the two USB connector positions.
        start = LIDAR_ASSEMBLY_X - h.HALF_X + 1
        end = h.TUNNEL_END_X - 1
        passage = Pos((start + end) / 2, LIDAR_Y,
                      LAYER_2_TOP + (h.HEIGHT - h.WALL) / 2) * Box(
            start - end, 2 * (h.HALF_Y - h.WALL), h.HEIGHT - h.WALL)
        for part in (housing, cover):
            overlap = part & passage
            self.assertLess(overlap.volume if overlap else 0, 1e-6)
        # Preserve solid material immediately above the notch.
        z = LAYER_2_TOP + h.HEIGHT + 0.5
        x = (chassis_common.SLOPE_END_X - chassis_common.SLOPE_COVER_SETBACK
             - (z - LAYER_2_TOP - chassis_slope_cover.LOW_RAIL_HEIGHT)
             * h.SLOPE_RUN / h.SLOPE_DROP)
        rim = Pos(x - 0.3, LIDAR_Y, z) * Box(0.2, 1, 0.2)
        self.assertAlmostEqual((cover & rim).volume, rim.volume)

    def test_housing_has_four_flat_interlocking_parts(self):
        panels = lidar_interface_housing.flat_parts()
        self.assertEqual(set(panels), {'lid', 'side_negative_y', 'side_positive_y', 'back'})
        for name, panel in panels.items():
            with self.subTest(panel=name):
                self.assertTrue(panel.is_valid)
                self.assertEqual(len(panel.solids()), 1)
                self.assertAlmostEqual(panel.bounding_box().size.Z,
                                       lidar_interface_housing.WALL)
        assembled = lidar_interface_housing.assemble(panels)
        self.assertTrue(assembled.is_valid)
        self.assertEqual(len(assembled.solids()), 6)
        self.assertAlmostEqual(assembled.volume,
                               sum(panel.volume for panel in panels.values())
                               + sum(post.volume for post in lidar_interface_housing.roof_spacers()))

    def test_two_foot_spacers_seat_on_deck_and_roof_with_aligned_bores(self):
        from itertools import combinations
        from robot_nira import chassis_diagonal_back, torso_layer_2
        from robot_nira.torso_layout import LAYER_2_BOTTOM, LAYER_2_TOP, LAYER_3_BOTTOM
        from utils import spacer
        from utils.colors import COLOR_WINE_RED

        h = lidar_interface_housing
        posts = h.roof_spacers()
        self.assertEqual(len(posts), 2)
        roof = Pos(Z=h.HEIGHT - h.WALL) * h.lid()
        # Express the body deck in housing coordinates, including its 180-degree rotation.
        deck = (Pos(Z=-torso_common.THICKNESS) * Rot(Z=180)
                * Pos(-LIDAR_ASSEMBLY_X, -LIDAR_Y)
                * torso_layer_2.build_model(torso_layer_2.build_surface()))
        for post, (x, y) in zip(posts, h.SPACER_POSITIONS):
            with self.subTest(position=(x, y)):
                self.assertTrue(post.is_valid)
                self.assertEqual(len(post.solids()), 1)
                self.assertAlmostEqual(post.bounding_box().min.Z, 0)
                self.assertAlmostEqual(post.bounding_box().max.Z, h.HEIGHT - h.WALL)
                self.assertAlmostEqual(post.bounding_box().size.X, spacer.FOOT_SPACER_OUTER_DIAMETER)
                self.assertAlmostEqual(post.volume, math.pi * (
                    (spacer.FOOT_SPACER_OUTER_DIAMETER / 2)**2
                    - (spacer.FOOT_SPACER_STUD_HOLE_DIAMETER / 2)**2) * h.SPACER_LENGTH)
                self.assertAlmostEqual(abs(y) - h.HALF_Y - h.SPACER_OUTER_DIAMETER / 2,
                                       h.SPACER_SIDE_CLEARANCE)
                self.assertEqual(tuple(post.color), tuple(COLOR_WINE_RED))
                top = max(post.faces(), key=lambda face: face.center().Z)
                self.assertEqual(len(top.outer_wire().edges()), 1)
                self.assertAlmostEqual(post.distance_to(roof), 0)
                self.assertAlmostEqual(post.distance_to(deck), 0)
                bore = Pos(x, y, (h.HEIGHT - torso_common.THICKNESS) / 2) * Cylinder(
                    h.SPACER_INNER_DIAMETER / 2, h.HEIGHT + torso_common.THICKNESS)
                for plate in (post, roof, deck):
                    overlap = plate & bore
                    self.assertLess(overlap.volume if overlap else 0, 1e-6)
                # Both plates support the complete annular footprint of each post.
                footprint = extrude(Face(top.outer_wire(), top.inner_wires()), amount=h.WALL)
                self.assertAlmostEqual((roof & footprint).volume, footprint.volume)
                foot = Pos(Z=-h.SPACER_LENGTH - h.WALL) * footprint
                self.assertAlmostEqual((deck & foot).volume, foot.volume)
        solids = h.build_model().solids()
        for a, b in combinations(solids, 2):
            overlap = a & b
            self.assertLess(overlap.volume if overlap else 0, 1e-6)
        housing = Pos(LIDAR_ASSEMBLY_X, LIDAR_Y, LAYER_2_TOP) * Rot(Z=180) * h.build_model()
        assembled_posts = [part for part in housing.children
                           if part.label.startswith('lidar_roof_spacer_')]
        self.assertEqual(len(assembled_posts), 2)
        for post in assembled_posts:
            self.assertEqual(tuple(post.color), tuple(COLOR_WINE_RED))
        for brace in chassis_diagonal_back.build_diagonal_plates(LAYER_2_BOTTOM, LAYER_3_BOTTOM):
            self.assertGreater(brace.distance_to(housing), 1.0, brace.label)
        # Existing callers retain their original cylindrical spacer geometry.
        original = spacer.build_model()
        self.assertAlmostEqual(original.volume, math.pi * (2**2 - 1.25**2) * 32)

    def test_wall_tabs_fill_lid_slots_without_protruding(self):
        panels = lidar_interface_housing.flat_parts()
        height = lidar_interface_housing.HEIGHT
        thickness = lidar_interface_housing.WALL
        lid = panels['lid']
        top = max(lid.faces(), key=lambda face: face.center().Z)
        # Enclosed finger slots plus two lidar bores and two spacer bores.
        tab_count = (2 * len(lidar_interface_housing.SIDE_LID_TAB_X)
                     + len(lidar_interface_housing.END_LID_TAB_Y))
        self.assertEqual(len(top.inner_wires()), tab_count + 4)
        for wire in top.inner_wires():
            self.assertGreater(wire.distance_to(top.outer_wire()), 1.0)
        for name in ('side_negative_y', 'side_positive_y', 'back'):
            with self.subTest(panel=name):
                panel = panels[name]
                bounds = panel.bounding_box()
                self.assertAlmostEqual(bounds.max.Y, height)
                band = Pos(bounds.center().X, height - thickness / 2,
                           thickness / 2) * Box(bounds.size.X, thickness, thickness)
                tabs = panel & band
                count = (len(lidar_interface_housing.SIDE_LID_TAB_X)
                         if name.startswith('side_') else len(lidar_interface_housing.END_LID_TAB_Y))
                self.assertEqual(len(tabs.solids()), count)
                self.assertAlmostEqual(tabs.volume,
                                       count * lidar_interface_housing.LID_TAB_WIDTH * thickness**2)

        assembled = lidar_interface_housing.assemble(panels)
        lid_band = Pos((lidar_interface_housing.MIN_X + lidar_interface_housing.MAX_X) / 2,
                       0, height - thickness / 2) * Box(
            lidar_interface_housing.MAX_X - lidar_interface_housing.MIN_X,
            2 * lidar_interface_housing.HALF_Y, thickness)
        placed_lid = Pos(Z=height - thickness) * lid
        # All wall tabs occupy the enclosed slots and touch the lid.
        # The volume check also detects tabs that overlap lid material.
        tab_count = (2 * len(lidar_interface_housing.SIDE_LID_TAB_X)
                     + len(lidar_interface_housing.END_LID_TAB_Y))
        expected_tab_volume = tab_count * lidar_interface_housing.LID_TAB_WIDTH * thickness**2
        wall_tabs = (assembled & lid_band) - placed_lid
        self.assertEqual(len(wall_tabs.solids()), tab_count)
        self.assertAlmostEqual(wall_tabs.volume, expected_tab_volume)
        for tab in wall_tabs.solids():
            self.assertLess(tab.distance_to(placed_lid), 1e-6)


class NiraLidarTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = build_assembly()
        cls.body = cls.robot.children[0].children[0]
        cls.parts = {part.label: part for part in cls.body.children}
        cls.lidar = cls.parts['lidar_ydlidar_tmini']

    def test_sensor_stack_clears_interface_board_in_front_third(self):
        from utils.colors import COLOR_WINE_RED
        deck = self.parts['torso_layer_2']
        interface = self.parts['board_ydlidar_tmini_interface']
        housing = self.parts['lidar_interface_housing']
        posts = [part for part in housing.children if part.label.startswith('lidar_roof_spacer_')]
        self.assertEqual(len(posts), 2)
        for post in posts:
            self.assertEqual(tuple(post.color), tuple(COLOR_WINE_RED))
        box = self.lidar.bounding_box()
        self.assertAlmostEqual(interface.bounding_box().min.Z, deck.bounding_box().max.Z)
        self.assertLess(interface.distance_to(deck), 1e-6)
        self.assertAlmostEqual(housing.bounding_box().min.Z, deck.bounding_box().min.Z)
        self.assertAlmostEqual(box.min.Z, housing.bounding_box().max.Z)
        self.assertLess(self.lidar.distance_to(housing), 1e-6)
        self.assertGreaterEqual(box.min.Z - lidar_interface_housing.WALL
                                - interface.bounding_box().max.Z, 1.0 - 1e-6)
        self.assertAlmostEqual(interface.joints['mount'].location.position.X
                               - lidar_interface_housing.CENTER_X, box.center().X)
        self.assertAlmostEqual(interface.joints['mount'].location.position.Y, box.center().Y)
        self.assertGreater(box.min.X, torso_common.rect_w / 6)
        self.assertAlmostEqual(box.center().X,
                               LIDAR_ASSEMBLY_X - lidar_interface_housing.CENTER_X)
        self.assertAlmostEqual(box.center().Y, LIDAR_Y)

    def test_canopy_rear_frame_and_roof_clear_scanner(self):
        rear_frame = self.parts['torso_layer_3']
        self.assertAlmostEqual(rear_frame.bounding_box().max.X, CANOPY_SHOULDER_X)
        for side in (-1, 1):
            support = Pos(CANOPY_SHOULDER_X - 1, side * (CANOPY_HALF_WIDTH - 2),
                          rear_frame.bounding_box().center().Z) * Box(1, 1, 1)
            self.assertAlmostEqual((support & rear_frame).volume, support.volume)
        for name in ('torso_layer_3', 'torso_layer_4'):
            with self.subTest(plate=name):
                plate = self.parts[name]
                # The rear frame is behind the sensor; its bounding box's Z
                # gap does not measure their physical clearance.
                self.assertGreater(plate.distance_to(self.lidar), 1e-6)
        canopy = self.parts['torso_layer_4']
        roof = canopy.location * canopy.children[0]
        # The roof overhang stays above the complete sensor envelope.
        self.assertGreater(roof.bounding_box().min.Z,
                           self.lidar.bounding_box().max.Z + 1e-6)
        self.assertAlmostEqual(roof.bounding_box().max.X, LIDAR_X)
        tip = Pos(LIDAR_X - 1, LIDAR_Y, roof.bounding_box().center().Z) * Box(1, 1, 1)
        self.assertAlmostEqual((tip & roof).volume, tip.volume)

    def test_rear_frame_retains_joint_slots_and_spacer_holes(self):
        frame = self.parts['torso_layer_3']
        z = frame.bounding_box().center().Z
        for location in [*torso_common.rectangle_slots_locations,
                         *torso_common.diagonal_slots_locations]:
            if location.position.X < 0:
                with self.subTest(slot=location):
                    probe = Pos(location.position.X, location.position.Y, z) * Box(1, 1, 1)
                    self.assertLess((probe & frame).volume, 1e-6)
        for location in torso_common.hole_locations:
            if location.position.X < 0:
                with self.subTest(hole=location):
                    bore = Pos(location.position.X, location.position.Y, z) * Cylinder(
                        torso_common.hole_radius / 2, torso_common.THICKNESS
                    )
                    self.assertLess((bore & frame).volume, 1e-6)
                    rim = Pos(location.position.X + torso_common.hole_radius + 1,
                              location.position.Y, z) * Box(1, 1, 1)
                    self.assertGreater((rim & frame).volume, 0)

    def test_housing_fingers_fit_deck_slots_and_retains_internal_connector_space(self):
        # Boolean operations on colored assembly children traverse parent links
        # and copy the robot tree. Test the placed solids without that metadata.
        housing = Part(self.parts['lidar_interface_housing'].wrapped)
        deck = Part(self.parts['torso_layer_2'].wrapped)
        interface = self.parts['board_ydlidar_tmini_interface']
        z = deck.bounding_box().center().Z
        self.assertEqual(len(lidar_interface_housing.DECK_TAB_POSITIONS), 6)
        for x in lidar_interface_housing.DECK_TAB_X:
            for y in (-lidar_interface_housing.DECK_SLOT_Y,
                      lidar_interface_housing.DECK_SLOT_Y):
                with self.subTest(x=x, y=y):
                    finger = Pos(LIDAR_ASSEMBLY_X + x, y, z) * Box(
                        lidar_interface_housing.DECK_TAB_WIDTH,
                        lidar_interface_housing.WALL,
                        torso_common.THICKNESS,
                    )
                    self.assertAlmostEqual((housing & finger).volume, finger.volume)
                    self.assertLess((deck & finger).volume, 1e-6)
        connector = interface.joints['connector'].location.position
        # The front is closed; retain the available plug space inside it.
        front_inner_x = LIDAR_ASSEMBLY_X - lidar_interface_housing.MIN_X - lidar_interface_housing.WALL
        depth = front_inner_x - connector.X - 1.0
        self.assertGreater(depth, 5.0)
        access = Pos(connector.X + depth / 2, connector.Y, connector.Z) * Box(depth, 10, 4)
        overlap = housing & access
        self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_modified_plates_are_valid_and_fit_the_body(self):
        # Strip assembly parent/joint references before boolean operations;
        # otherwise build123d deep-copies the entire robot for each pair.
        shapes = {part.label: Part(part.wrapped) for part in self.body.children}
        for part in self.body.children:
            with self.subTest(part=part.label):
                self.assertTrue(part.is_valid)
                for component in leaves(part):
                    expected = 6 if component.label == 'lidar_interface_housing' else 1
                    self.assertEqual(len(component.solids()), expected, component.label)
            if not part.label.startswith(('chassis_', 'lidar_')):
                continue
            for other in self.body.children:
                if other is part:
                    continue
                with self.subTest(part=part.label, other=other.label):
                    overlap = shapes[part.label] & shapes[other.label]
                    self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_retained_upper_tabs_engage_layer_3(self):
        top = self.parts['torso_layer_3'].bounding_box()
        tab_slice = Pos(0, 0, top.center().Z) * Box(300, 200, torso_common.THICKNESS)
        for part in self.body.children:
            if not part.label.startswith('chassis_') or part.label == 'chassis_slope_cover':
                continue
            with self.subTest(part=part.label):
                tabs = part & tab_slice
                if 'front' in part.label:
                    self.assertLess(tabs.volume if tabs else 0, 1e-6)
                else:
                    self.assertEqual(len(tabs.solids()), 3)
                    self.assertAlmostEqual(
                        tabs.volume, 3 * chassis_common.TAB_WIDTH
                        * chassis_common.THICKNESS * torso_common.THICKNESS)
                    # The tab footprints sit inside the canopy perimeter.
                    with BuildPart() as outline:
                        with BuildSketch(Plane.XY.offset(top.min.Z)):
                            insert(canopy_surface())
                        extrude(amount=torso_common.THICKNESS)
                    self.assertAlmostEqual((tabs & outline.part).volume, tabs.volume)

    def test_front_plates_fit_their_deck_fingers(self):
        deck = self.parts['torso_layer_2']
        deck_box = deck.bounding_box()
        z = deck_box.center().Z
        front = self.parts['chassis_front']
        self.assertAlmostEqual(front.bounding_box().max.Z, deck_box.max.Z)
        fingers = front & Pos(0, 0, z) * Box(300, 200, torso_common.THICKNESS)
        self.assertEqual(len(fingers.solids()), 3)
        self.assertAlmostEqual(fingers.volume,
                               3 * chassis_common.TAB_WIDTH * chassis_common.THICKNESS
                               * torso_common.THICKNESS)
        self.assertLess((fingers & deck).volume, 1e-6)
        for name in ('chassis_diagonal_front_left', 'chassis_diagonal_front_right'):
            self.assertNotIn(name, self.parts)

        for y in (-chassis_common.TAB_PITCH, 0, chassis_common.TAB_PITCH):
            slot = Pos(100, y, z) * Box(torso_common.rectangle_slots_height,
                                         chassis_common.TAB_WIDTH, torso_common.THICKNESS)
            self.assertLess((slot & deck).volume, 1e-6)
        for y in (-chassis_common.TAB_PITCH / 2, chassis_common.TAB_PITCH / 2):
            web = Pos(100, y, z) * Box(torso_common.rectangle_slots_height,
                                        torso_common.rectangle_slots_height,
                                        torso_common.THICKNESS)
            self.assertAlmostEqual((web & deck).volume, web.volume)

    def test_slope_cover_closes_gap_between_side_plates(self):
        cover = self.parts['chassis_slope_cover']
        left = self.parts['chassis_left']
        right = self.parts['chassis_right']
        top = self.parts['torso_layer_3'].bounding_box().min.Z

        self.assertTrue(cover.is_valid)
        self.assertEqual(len(cover.solids()), 1)
        self.assertAlmostEqual(cover.bounding_box().max.X,
                               chassis_common.SLOPE_COVER_END_X - chassis_common.SLOPE_COVER_SETBACK)
        self.assertAlmostEqual(cover.bounding_box().min.Z,
                               self.parts['torso_layer_2'].bounding_box().max.Z)
        self.assertAlmostEqual(cover.bounding_box().max.Z, top)
        for side in (left, right):
            self.assertLess(cover.distance_to(side), 1e-6)
            overlap = cover & side
            self.assertLess(overlap.volume if overlap else 0, 1e-6)
            # Five cover fingers occupy enclosed slots across the complete
            # side-plate thickness.
            side_band = Pos(0, side.bounding_box().center().Y, top - 20) * Box(
                300, chassis_common.THICKNESS, 100)
            fingers = cover & side_band
            self.assertEqual(len(fingers.solids()), chassis_common.JOINT_FINGER_COUNT)
            self.assertAlmostEqual(
                fingers.volume,
                chassis_common.JOINT_FINGER_COUNT
                * chassis_common.JOINT_FINGER_LENGTH
                * chassis_common.THICKNESS ** 2,
            )
            # Every tongue is surrounded by side material, rather than opening
            # onto the sloped perimeter.
            face = max(side.faces(), key=lambda face: face.area)
            holes = face.inner_wires()
            self.assertGreaterEqual(len(holes), chassis_common.JOINT_FINGER_COUNT)
            for finger in fingers.solids():
                self.assertTrue(any(
                    (wire.bounding_box().center() - finger.center()).length < chassis_common.THICKNESS
                    for wire in holes
                ))
        # The cover spans the centre at mid-slope while clearing the housing
        # at its lower edge.
        midpoint = (chassis_common.SLOPE_START_X + chassis_common.SLOPE_END_X) / 2 - 3.0
        # Derive the centre from the cover ends, then probe along the slope's
        # normal inside the sheet. This remains valid when its angle changes.
        low_top = self.parts['torso_layer_2'].bounding_box().max.Z + chassis_slope_cover.LOW_RAIL_HEIGHT
        run = chassis_common.SLOPE_END_X - chassis_common.SLOPE_START_X
        drop = top - low_top
        plane = Plane(origin=(midpoint, 0, (top + low_top) / 2),
                      x_dir=(0, 1, 0), z_dir=(drop, 0, run))
        probe = plane.location * Pos(Z=-chassis_common.THICKNESS / 2) * Box(
            1, 1, chassis_common.THICKNESS / 2)
        self.assertAlmostEqual((cover & probe).volume, probe.volume)
        housing_overlap = Part(cover.wrapped) & Part(self.parts['lidar_interface_housing'].wrapped)
        self.assertLess(housing_overlap.volume if housing_overlap else 0, 1e-6)

    def test_two_speakers_face_perforations_from_inside(self):
        cover = self.parts['chassis_slope_cover']
        slots = chassis_slope_cover.speaker_slot_positions()
        self.assertGreater(len(slots), 20)

        for name, y in zip(('left', 'right'), chassis_slope_cover.SPEAKER_Y_POSITIONS):
            speaker = self.parts[f'loudspeaker_{name}']
            with self.subTest(speaker=name):
                self.assertTrue(speaker.is_valid)
                self.assertEqual(len(speaker.solids()), 1)
                self.assertAlmostEqual(speaker.bounding_box().center().Y, y)
                self.assertLess(speaker.distance_to(cover), 1e-6)
                overlap = speaker & cover
                self.assertLess(overlap.volume if overlap else 0, 1e-6)
                plane = chassis_slope_cover.speaker_plane(
                    self.parts['torso_layer_3'].bounding_box().min.Z, y)
                for across, along in (slots[0], (0, 0), slots[-1]):
                    probe = (plane.location
                             * Pos(across, along, -chassis_common.THICKNESS / 2)
                             * Cylinder(0.1,
                                        chassis_common.THICKNESS + 0.2))
                    self.assertFalse(cover & probe)
                # Probe the central octagon and the solid bridges halfway
                # to its neighbouring openings.
                for dx, dy, is_open in ((1, 1, True), (-1, -1, True),
                                        (2.5, 0, False), (-2.5, 0, False)):
                    probe = (plane.location
                             * Pos(dx, dy, -chassis_common.THICKNESS / 2)
                             * Cylinder(0.1, chassis_common.THICKNESS + 0.2))
                    self.assertEqual((cover & probe).volume < 1e-6, is_open)

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
