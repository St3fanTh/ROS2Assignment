import unittest
from movelt_planning_obstacle  import CartesianInterpUR5, interpolate_pose
from unittest.mock import MagicMock

class TestInterpolation(unittest.TestCase):
    def test_linear_interpolation(self):
        start = [0, 0, 0, 0, 0, 0]
        end = [1, 1, 1, 0, 0, 0]
        steps = 5
        result = interpolate_pose(start, end, steps)
        self.assertEqual(len(result), steps)
        self.assertEqual(result[0], start)
        self.assertEqual(result[-1], end)
