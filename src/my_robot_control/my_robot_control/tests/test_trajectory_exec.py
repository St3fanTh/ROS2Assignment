class TestTrajectoryExecution(unittest.TestCase):
    def setUp(self):
        self.node = CartesianInterpUR5([0.2, 0.2, 0.2, 0, 0, 0])
        self.node.wait_for_joint_state = MagicMock(return_value=True)
        self.node.wait_for_services = MagicMock(return_value=True)
        self.node.current_pose = MagicMock(return_value=MagicMock(
            pose=MagicMock(
                position=MagicMock(x=0, y=0, z=0),
                orientation=MagicMock(x=0, y=0, z=0, w=1)
            )
        ))
        self.node.request_ik = MagicMock(return_value=[0.0]*6)
        self.node.act_client.send_goal_async = MagicMock()

    def test_execute_creates_markers_and_trajectory(self):
        self.node.marker_pub.publish = MagicMock()
        self.node.execute()
        self.assertTrue(self.node.marker_pub.publish.called)
        self.assertFalse(self.node.error_occurred)
