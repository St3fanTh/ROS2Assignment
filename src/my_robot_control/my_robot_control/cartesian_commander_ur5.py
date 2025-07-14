#!/usr/bin/env python3
"""

Moves the UR5 arm to a Cartesian target pose (x y z roll pitch yaw):

    ros2 run my_robot_control cartesian_commander_ur5 0.4 0.2 0.3 0 1.57 0

• IK service:   /compute_ik   (MoveIt)
• Controller:   /ur5_arm_controller/follow_joint_trajectory
• Joint names:  shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
"""

import sys
import math
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


# ------------------------------------------------------------
# Helper: request inverse kinematics
# ------------------------------------------------------------
def request_ik(node: Node, pose: PoseStamped, group: str = "ur5_arm"):
    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /compute_ik not available!")
        return None

    req = GetPositionIK.Request()
    req.ik_request.group_name = group
    req.ik_request.pose_stamped = pose
    req.ik_request.timeout = Duration(seconds=5).to_msg()
    req.ik_request.avoid_collisions = True

    # Seed state (home pose)
    js = JointState()
    js.name = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    js.position = [0.0] * 6
    req.ik_request.robot_state.joint_state = js

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    if res.error_code.val != res.error_code.SUCCESS:
        node.get_logger().error(f"IK failed (Error code {res.error_code.val})")
        return None

    ik_map = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [ik_map[name] for name in js.name]


# ------------------------------------------------------------
# Main node
# ------------------------------------------------------------
class CartesianCommanderUR5(Node):
    def __init__(self, pose_xyzrpy):
        super().__init__("cartesian_commander_ur5")
        self.pose_xyzrpy = pose_xyzrpy

        # Action client for FollowJointTrajectory
        self.act_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/ur5_arm_controller/follow_joint_trajectory",
        )
        self.act_client.wait_for_server()

        self.send_pose()

    # --------------------------------------------------------
    def send_pose(self):
        x, y, z, roll, pitch, yaw = self.pose_xyzrpy
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # 1) Compute IK
        joint_positions = request_ik(self, pose)
        if joint_positions is None:
            self.get_logger().error("Aborting: IK did not return a solution.")
            return
        self.get_logger().info(
            "IK joint angles: " + ", ".join(f"{j:+.3f}" for j in joint_positions)
        )

        # 2) Build trajectory goal
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = joint_positions
        pt.time_from_start.sec = 3  # 3-second trajectory
        traj.points.append(pt)
        goal.trajectory = traj

        # 3) Send goal & wait for result
        self.get_logger().info("Sending trajectory to controller…")
        send_future = self.act_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal was rejected by the controller!")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅ Trajectory executed successfully")
        else:
            self.get_logger().error(
                f"❌ Execution failed (Error code {result.error_code})"
            )


# ------------------------------------------------------------
# CLI entry point
# ------------------------------------------------------------
def main(argv=sys.argv[1:]):
    if len(argv) != 6:
        print(
            "Usage:\n  ros2 run my_robot_control cartesian_commander_ur5 "
            "<x> <y> <z> <roll> <pitch> <yaw>"
        )
        return

    pose_vals = [float(v) for v in argv]
    rclpy.init()
    node = CartesianCommanderUR5(pose_vals)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
