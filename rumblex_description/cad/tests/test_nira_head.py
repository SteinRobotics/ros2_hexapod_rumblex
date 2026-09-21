"""Camera support, mounting bores and retained head joint interfaces."""

import unittest

from build123d import Cylinder, Pos, Rot

from robot_nira import assembly_head as head
from robot_nira import webcam_obsbot


class NiraHeadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assembly = head.build_assembly()
        cls.parts = {part.label: part for part in cls.assembly.children}

    def test_new_parts_are_single_solids_without_interference(self):
        parts = list(self.parts.values())
        for i, part in enumerate(parts):
            self.assertTrue(part.is_valid, part.label)
            self.assertEqual(len(part.solids()), 1, part.label)
            for other in parts[i + 1:]:
                # The vendor bracket's empirical servo registration predates
                # this cage; validate every pair involving a new component.
                if {part.label, other.label} == {"head_pitch_servo", "head_side_bracket"}:
                    continue
                overlap = part & other
                self.assertLess(overlap.volume if overlap else 0, 1e-6,
                                f"{part.label} / {other.label}")

    def test_camera_seats_on_chin_with_accessible_tripod_bore(self):
        camera = self.parts["webcam_obsbot"]
        chin = self.parts["head_chin"]
        self.assertAlmostEqual(camera.bounding_box().min.Z, chin.bounding_box().max.Z)
        mount = camera.joints["mount"].location
        tool = mount * Pos(Z=-head.PLATE_THICKNESS / 2) * Cylinder(
            webcam_obsbot.MOUNT_HOLE_DIAMETER / 2, head.PLATE_THICKNESS)
        overlap = tool & chin
        self.assertLess(overlap.volume if overlap else 0, 1e-6)
        self.assertLess(camera.distance_to(chin), 1e-6)
        # Everything stays behind the optical face, including its recess.
        lens_x = camera.joints["lens"].location.position.X
        for part in self.parts.values():
            if part is not camera:
                self.assertLess(part.bounding_box().max.X, lens_x)

    def test_standoffs_seat_on_plates_and_match_bracket_bores(self):
        for i in range(4):
            post = self.parts[f"head_cage_spacer_{i}"]
            for label in ("head_chin", "head_brow"):
                self.assertLess(post.distance_to(self.parts[label]), 1e-6)
        bracket = self.parts["head_side_bracket"]
        adapter = self.parts["head_rear_adapter"]
        self.assertLess(adapter.distance_to(bracket), 1e-6)
        self.assertAlmostEqual(adapter.bounding_box().min.X, bracket.bounding_box().max.X)
        self.assertFalse(any(name.startswith("head_bracket_spacer_") for name in self.parts))
        # Adapter and vendor bracket share the same four through bores.
        for y in (-head.BRACKET_HOLE_HALF_PITCH, head.BRACKET_HOLE_HALF_PITCH):
            for z in (-head.BRACKET_HOLE_HALF_PITCH, head.BRACKET_HOLE_HALF_PITCH):
                bore = Pos(bracket.bounding_box().max.X + (head.PLATE_THICKNESS - 1.5) / 2,
                           head.BRACKET_CENTER_Y + y, head.BRACKET_CENTER_Z + z) * Rot(Y=90) * Cylinder(
                               1.59, head.PLATE_THICKNESS + 1.5)
                for part in (bracket, adapter):
                    overlap = bore & part
                    self.assertLess(overlap.volume if overlap else 0, 1e-6)

    def test_omitting_servo_preserves_joint_frames(self):
        without = head.build_assembly(include_servo=False)
        self.assertNotIn("head_pitch_servo", [part.label for part in without.children])
        for name in ("servo_mount", "pitch"):
            self.assertEqual(self.assembly.joints[name].location, without.joints[name].location)


if __name__ == "__main__":
    unittest.main()
