import unittest
from unittest.mock import MagicMock
from movelt_planning_obstacle  import CartesianInterpUR5

class TestUR5Setup(unittest.TestCase):
    def setUp(self):
        self.pose = [0.4, 0.4, 0.2, 0.5, 1.57, 0.0]
        self.node = CartesianInterpUR5(self.pose)

    def test_home_position_format(self):
        self.assertEqual(len(self.node.home_position), 6)
        for angle in self.node.home_position:
            self.assertIsInstance(angle, float)

    def test_joint_names_length(self):
        self.assertEqual(len(self.node.current_joint_state.name), 6)
