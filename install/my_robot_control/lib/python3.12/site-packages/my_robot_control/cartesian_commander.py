#!/usr/bin/env python3
"""
cartesian_commander.py

Bewegt den Panda-Arm zu einer kartesischen Zielpose (x y z R P Y),
nutzt MoveIt's IK-Service '/compute_ik' und sendet die Trajektorie
an den Controller '/panda_arm_controller/follow_joint_trajectory'.
"""
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler

# ------------------------------------------------------------
# Hilfsfunktion: IK anfragen
# ------------------------------------------------------------
def request_ik(node: Node, pose: PoseStamped, group: str = "panda_arm"):
    cli = node.create_client(GetPositionIK, "/compute_ik")
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /compute_ik nicht verfügbar!")
        return None

    req = GetPositionIK.Request()
    req.ik_request.group_name = group
    req.ik_request.pose_stamped = pose
    req.ik_request.timeout = Duration(seconds=5).to_msg()

    # Seed-State setzen (Startposition für IK)
    js = JointState()
    js.name = [
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    ]
    js.position = [0.0] * 7  # Startpose: "Nullstellung" – kann angepasst werden
    req.ik_request.robot_state.joint_state = js

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    if res.error_code.val != res.error_code.SUCCESS:
        node.get_logger().error(f"IK fehlgeschlagen (Code {res.error_code.val})")
        return None

    # Gelenkwerte herausfiltern (nur die Panda-Gelenke)
    joint_names = js.name
    ik_dict = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    filtered_positions = [ik_dict.get(name, 0.0) for name in joint_names]

    return filtered_positions

# ------------------------------------------------------------
# Haupt-Node
# ------------------------------------------------------------
class CartesianCommander(Node):
    def __init__(self, pose_xyzrpy):
        super().__init__("cartesian_commander")
        self.pose_xyzrpy = pose_xyzrpy

        # Action-Client starten
        self.act_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )
        self.act_client.wait_for_server()

        # Pose → IK → Trajektorie
        self.send_pose()

    def send_pose(self):
        x, y, z, roll, pitch, yaw = self.pose_xyzrpy
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, \
        pose.pose.orientation.z, pose.pose.orientation.w = q

        # 1) IK anfragen
        joint_positions = request_ik(self, pose)
        if joint_positions is None:
            self.get_logger().error("Abbruch: IK lieferte keine Lösung.")
            return
        self.get_logger().info(f"IK-Winkel: {[round(j, 3) for j in joint_positions]}")

        # 2) Trajektorie bauen
        joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
        ]

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = joint_positions
        pt.time_from_start.sec = 3
        traj.points.append(pt)
        goal.trajectory = traj

        self.get_logger().info("Sende Trajektorie an Controller …")
        for name, val in zip(joint_names, joint_positions):
            self.get_logger().info(f"  {name}: {round(val, 3)} rad")

        # 3) Ziel senden & warten
        future = self.act_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Ziel wurde vom Controller abgelehnt!")
            return

        self.get_logger().info("Ziel akzeptiert. Warte auf Ausführung …")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_string:
            self.get_logger().warn(f"Fehlerstring vom Controller: {result.result.error_string}")
        else:
            self.get_logger().info("Trajektorie erfolgreich ausgeführt ✅")

# ------------------------------------------------------------------
# CLI-Einstieg
# ------------------------------------------------------------------
def main(argv=sys.argv[1:]):
    if len(argv) != 6:
        print("Aufruf: ros2 run my_robot_control cartesian_commander <x> <y> <z> <roll> <pitch> <yaw>")
        return

    pose_vals = [float(v) for v in argv]
    rclpy.init()
    node = CartesianCommander(pose_vals)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
