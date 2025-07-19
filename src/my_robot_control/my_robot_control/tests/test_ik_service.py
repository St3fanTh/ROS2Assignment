from geometry_msgs.msg import PoseStamped
import unittest
from movelt_planning_obstacle  import CartesianInterpUR5
from unittest.mock import MagicMock

class TestIKService(unittest.TestCase):
    def setUp(self):
        self.node = CartesianInterpUR5([0.1, 0.1, 0.1, 0, 0, 0])
        self.node.ik_client.call_async = MagicMock()

    def test_request_ik_valid(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.1
        pose.pose.position.y = 0.2
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0

        mock_future = MagicMock()
        mock_future.result.return_value = MagicMock(
            error_code=MagicMock(val=1),
            solution=MagicMock(joint_state=MagicMock(
                name=self.node.current_joint_state.name,
                position=[0.1]*6
            ))
        )
        self.node.ik_client.call_async.return_value = mock_future
        result = self.node.request_ik(pose)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 6)
