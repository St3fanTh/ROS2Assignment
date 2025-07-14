#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HomeCommander(Node):
    def __init__(self):
        super().__init__("home_commander")

        # Create an action client for the FollowJointTrajectory action
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )

        # Wait until the action server is available
        self.client.wait_for_server()
        # Send the robot to its "home" position
        self.send_home()

    def send_home(self):
        # Build the trajectory goal to move to the home configuration
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
        ]

        # Define one trajectory point: the home joint angles
        point = JointTrajectoryPoint()
        point.positions = [
            0.0,
            -math.pi / 4,   # -45°
            0.0,
            -3 * math.pi / 4,  # -135°
            0.0,
            math.pi / 2,   # 90°
            math.pi / 4    # 45°
        ]
        point.time_from_start.sec = 3  # 3-second motion
        traj.points.append(point)

        # Attach the trajectory to the goal
        goal.trajectory = traj

        # Send the goal and wait for completion
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        result = send_future.result()

        if result:
            self.get_logger().info("Robot returned to home position ✅")
        else:
            self.get_logger().error("Failed to send trajectory to controller!")

def main():
    rclpy.init()
    node = HomeCommander()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
