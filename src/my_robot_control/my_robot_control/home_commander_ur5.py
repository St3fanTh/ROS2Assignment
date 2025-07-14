#!/usr/bin/env python3
"""
Sends a trajectory to the UR5 joint trajectory controller
to move the robot into its home position.

• Tested on ROS 2 Jazzy with ros2_control & MoveIt
• Action server: /ur5_arm_controller/follow_joint_trajectory
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HomeCommander(Node):
    def __init__(self):
        super().__init__("ur5_home_commander")

        # ➊ Initialize the action client
        self.cli = ActionClient(
            self,
            FollowJointTrajectory,
            "/ur5_arm_controller/follow_joint_trajectory",
        )

        self.get_logger().info("Waiting for /ur5_arm_controller…")
        self.cli.wait_for_server()

        # ➋ Build and send the home trajectory
        self.send_home()

    # ------------------------------------------------------------
    def send_home(self):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()

        # UR5 joint names (in the same order as in the URDF / joint_states)
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Typical home angles (radians); adjust as needed
        home_pos = [
            0.0,           # shoulder_pan
            -math.pi / 2,  # shoulder_lift
            math.pi / 2,   # elbow
            -math.pi / 2,  # wrist_1
            -math.pi / 2,  # wrist_2
            0.0,           # wrist_3
        ]

        pt = JointTrajectoryPoint()
        pt.positions = home_pos
        pt.time_from_start.sec = 3   # 3 seconds to reach home
        pt.time_from_start.nanosec = 0
        traj.points.append(pt)

        goal_msg.trajectory = traj

        # ➌ Send the trajectory asynchronously
        send_future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        if not send_future.result():
            self.get_logger().error("❌  Failed to send the goal!")
            return

        result_future = send_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅  UR5 is now in home position")
        else:
            self.get_logger().error(
                f"❌  Trajectory execution failed (Error code {result.error_code})"
            )


def main():
    rclpy.init()
    node = HomeCommander()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
