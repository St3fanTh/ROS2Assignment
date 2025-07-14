#!/usr/bin/env python3
"""
cartesian_commander_ur5_interp.py
Linear-interpolierte Cartesian-Bewegung mit Kollisionsprüfung.
Aufrufbeispiel:
  ros2 run my_robot_control cartesian_commander_ur5_interp 0.4 0.2 0.3 0 1.57 0
"""

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from tf_transformations import quaternion_from_euler


# ------------------------------------------------------
# IK-Service
# ------------------------------------------------------
def request_ik(node: Node, pose: PoseStamped, group: str = "ur5_arm"):
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service()

    req = GetPositionIK.Request()
    req.ik_request.group_name = group
    req.ik_request.pose_stamped = pose
    req.ik_request.timeout = Duration(seconds=2).to_msg()
    req.ik_request.avoid_collisions = True

    js = JointState()
    js.name = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]
    js.position = [0.0] * 6
    req.ik_request.robot_state.joint_state = js

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    if res.error_code.val != res.error_code.SUCCESS:
        return None

    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [sol[n] for n in js.name]


# ------------------------------------------------------
# Haupt-Node
# ------------------------------------------------------
class CartesianInterpUR5(Node):
    def __init__(self, target_xyzrpy, steps=20):
        super().__init__("cartesian_commander_ur5_interp")

        self.steps = steps
        self.target = target_xyzrpy

        self.act_client = ActionClient(
            self, FollowJointTrajectory,
            "/ur5_arm_controller/follow_joint_trajectory")
        self.act_client.wait_for_server()

        self.plan_and_execute()

    # --------------------------------------------------
    def current_pose(self):
        # Quick-&-dirty Startpose (Greifer über Home) – in echtem System per FK lesen
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.4
        pose.pose.orientation.w = 1.0
        return pose

    # --------------------------------------------------
    def plan_and_execute(self):
        start_pose = self.current_pose()
        x0, y0, z0 = start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z
        roll0, pitch0, yaw0 = 0.0, 0.0, 0.0   # Annahme

        x1, y1, z1, roll1, pitch1, yaw1 = self.target

        # Cartesian-Interpolation
        cart_points = [
            [x0 + (x1 - x0) * t,
             y0 + (y1 - y0) * t,
             z0 + (z1 - z0) * t,
             roll0 + (roll1 - roll0) * t,
             pitch0 + (pitch1 - pitch0) * t,
             yaw0 + (yaw1 - yaw0) * t]
            for t in np.linspace(0.0, 1.0, self.steps)
        ]

        joint_traj = JointTrajectory()
        joint_traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        for i, pt_xyzrpy in enumerate(cart_points):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pt_xyzrpy[:3]
            q = quaternion_from_euler(*pt_xyzrpy[3:])
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q

            joints = request_ik(self, pose)
            if joints is None:
                self.get_logger().error(f"❌ IK-Fehlschlag bei Wegpunkt {i}. Abbruch.")
                return

            jp = JointTrajectoryPoint()
            jp.positions = joints
            jp.time_from_start = Duration(seconds=3.0 * i / (self.steps - 1)).to_msg()
            joint_traj.points.append(jp)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info("Sende Trajektorie ({} Punkte) …".format(len(joint_traj.points)))
        future = self.act_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()

        if not gh.accepted:
            self.get_logger().error("❌ Trajektorie vom Controller abgelehnt!")
            return

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅ Bewegung erfolgreich ausgeführt")
        else:
            self.get_logger().error(f"❌ Controller-Fehler (Code {res.error_code})")


# ------------------------------------------------------
def main(argv=sys.argv[1:]):
    if len(argv) != 6:
        print("Aufruf:\n  ros2 run my_robot_control cartesian_commander_ur5_interp "
              "<x> <y> <z> <roll> <pitch> <yaw>")
        return

    target = [float(v) for v in argv]
    rclpy.init()
    node = CartesianInterpUR5(target, steps=50)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
