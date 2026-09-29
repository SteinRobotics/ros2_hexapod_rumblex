"""Regression checks for offline commanded robot motion."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from planar_motion import integrate_pose


class PlanarMotionTest(unittest.TestCase):
    def assert_pose(self, actual, expected):
        for value, target in zip(actual, expected):
            self.assertAlmostEqual(value, target)

    def test_forward(self):
        self.assert_pose(integrate_pose(0, 0, 0, 0.2, 0, 0, 5), (1, 0, 0))

    def test_body_frame_translation(self):
        self.assert_pose(integrate_pose(0, 0, math.pi / 2, 0.2, 0.1, 0, 5),
                         (-0.5, 1, math.pi / 2))

    def test_turning_arc(self):
        self.assert_pose(integrate_pose(0, 0, 0, 1, 0, 1, math.pi / 2),
                         (1, 1, math.pi / 2))

    def test_rotation_in_place(self):
        self.assert_pose(integrate_pose(1, 2, 0, 0, 0, -1, math.pi / 2),
                         (1, 2, -math.pi / 2))

    def test_stop_and_clock_reset(self):
        self.assert_pose(integrate_pose(1, 2, 0.5, 0, 0, 0, 5), (1, 2, 0.5))
        self.assert_pose(integrate_pose(1, 2, 0.5, 1, 1, 1, -1), (1, 2, 0.5))

    def test_timer_rate_independence(self):
        pose = (0, 0, 0)
        for _ in range(100):
            pose = integrate_pose(*pose, 0.2, -0.1, 0.3, 0.01)
        self.assert_pose(pose, integrate_pose(0, 0, 0, 0.2, -0.1, 0.3, 1))


if __name__ == '__main__':
    unittest.main()
