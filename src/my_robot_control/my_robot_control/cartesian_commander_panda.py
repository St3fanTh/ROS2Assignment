#!/usr/bin/env python3

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
# Helper function: request inverse kinematics
# ------------------------------------------------------------
def request_ik(node: Node, pose: PoseStamped, group: str = "panda_arm"):
    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /compute_ik not available!")
        return None

    req = GetPositionIK.Request()
    req.ik_request.group_name = group
    req.ik_request.pose_stamped = pose
    req.ik_request.timeout = Duration(seconds=5).to_msg()

    # Set seed state (start position for IK)
    js = JointState()
    js.name = [
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    ]
    js.position = [0.0] * 7  # Home pose: all zeros (adjustable)
    req.ik_request.robot_state.joint_state = js

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    if res.error_code.val != res.error_code.SUCCESS:
        node.get_logger().error(f"IK failed (Error code {res.error_code.val})")
        return None

    # Extract joint values (only Panda arm joints)
    joint_names = js.name
    ik_map = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    filtered_positions = [ik_map.get(name, 0.0) for name in joint_names]

    return filtered_positions

# ------------------------------------------------------------
# Main node: sends Cartesian pose as a joint trajectory
# ------------------------------------------------------------
class CartesianCommander(Node):
    def __init__(self, pose_xyzrpy):
        super().__init__("cartesian_commander")
        self.pose_xyzrpy = pose_xyzrpy

        # Publisher for raw JointTrajectory messages
        self.traj_pub = self.create_publisher(JointTrajectory, "/arm_trajectory", 10)

        # Action client for the FollowJointTrajectory action
        self.act_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )
        self.act_client.wait_for_server()

        # Immediately send the pose
        self.send_pose()

    def send_pose(self):
        x, y, z, roll, pitch, yaw = self.pose_xyzrpy

        # Build the PoseStamped in the world frame
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        (pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w) = q

        # 1) Request IK
        joint_positions = request_ik(self, pose)
        if joint_positions is None:
            self.get_logger().error("Aborting: IK did not return a solution.")
            return
        self.get_logger().info(f"IK joint angles: {[round(j, 3) for j in joint_positions]}")

        # 2) Build the trajectory message
        joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
        ]
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 3  # 3-second motion
        traj.points.append(point)

        # Publish the raw trajectory (optional)
        self.traj_pub.publish(traj)

        # Prepare and send the FollowJointTrajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info("Sending trajectory to controller…")
        for name, val in zip(joint_names, joint_positions):
            self.get_logger().info(f"  {name}: {round(val, 3)} rad")

        # 3) Send goal and wait for execution
        send_future = self.act_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the controller!")
            return

        self.get_logger().info("Goal accepted. Waiting for execution…")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_string:
            self.get_logger().warn(f"Controller error string: {result.result.error_string}")
        else:
            self.get_logger().info("Trajectory executed successfully ✅")

# ------------------------------------------------------------------
# CLI entry point
# ------------------------------------------------------------------
def main(argv=sys.argv[1:]):
    if len(argv) != 6:
        print("Usage: ros2 run my_robot_control cartesian_commander <x> <y> <z> <roll> <pitch> <yaw>")
        return

    # Parse the 6 pose parameters
    pose_vals = [float(v) for v in argv]
    rclpy.init()
    node = CartesianCommander(pose_vals)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
