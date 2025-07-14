#!/usr/bin/env python3
"""
Sendet dem UR5-Joint-Trajectory-Controller eine Trajektorie,
um den Roboter in die Home-Position zu fahren.

• Getestet unter ROS 2 Jazzy, ros2_control & MoveIt
• Action-Server:  /ur5_arm_controller/follow_joint_trajectory
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

        # ➊ Action-Client initialisieren
        self.cli = ActionClient(
            self,
            FollowJointTrajectory,
            "/ur5_arm_controller/follow_joint_trajectory",
        )

        self.get_logger().info("Warte auf /ur5_arm_controller …")
        self.cli.wait_for_server()

        # ➋ Home-Trajektorie bauen & senden
        self.send_home()

    # ------------------------------------------------------------
    def send_home(self):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()

        # UR5-Gelenknamen (Reihenfolge wie im URDF / joint_states)
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Typische Home-Winkel (Radiant); gern anpassen
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
        pt.time_from_start.sec = 3          # 3 s bis Home
        pt.time_from_start.nanosec = 0
        traj.points.append(pt)

        goal_msg.trajectory = traj

        # ➌ Trajektorie asynchron senden
        send_future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        if not send_future.result():
            self.get_logger().error("❌  Goal konnte nicht gesendet werden!")
            return

        result_future = send_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅  UR5 steht in Home-Position")
        else:
            self.get_logger().error(f"❌  Fehler beim Ausführen der Trajektorie (Code {result.error_code})")


def main():
    rclpy.init()
    node = HomeCommander()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
