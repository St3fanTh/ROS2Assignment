#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class HomeCommander(Node):
    def __init__(self):
        super().__init__("home_commander")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )

        self.client.wait_for_server()
        self.send_home()

    def send_home(self):
        # Trajektorie zum "Home"-Zustand
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            0.0, -math.pi / 4, 0.0, -3 * math.pi / 4,
            0.0, math.pi / 2, math.pi / 4
        ]
        point.time_from_start.sec = 3
        traj.points.append(point)
        goal.trajectory = traj

        self.future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)
        result = self.future.result()
        if result:
            self.get_logger().info("Roboter zurück in Home-Position ✅")
        else:
            self.get_logger().error("Fehler beim Senden der Trajektorie!")

def main():
    rclpy.init()
    node = HomeCommander()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
