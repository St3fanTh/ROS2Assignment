#!/usr/bin/env python3

"""

Moves the UR5 arm to a Cartesian target pose (x y z roll pitch yaw) and computes/visualize the trajectory with linear interpolation:

    ros2 run my_robot_control movelt_planning_obstacle 0.4 0.4 0.2 0.5 1.57 0

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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import time
import os


def interpolate_pose(start, goal, steps):
    """Linear interpolation between two Cartesian poses (6 DoF)."""
    interpolated = []
    for i in range(steps):
        alpha = i / (steps - 1)
        point = [(1 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        interpolated.append(point)
    return interpolated


class CartesianInterpUR5(Node):
    def __init__(self, pose_xyzrpy):
        super().__init__("cartesian_commander_ur5_interp")
        self.pose_xyzrpy = pose_xyzrpy
        self.execution_complete = False
        self.error_occurred = False
        self.has_joint_state = False  # Flag for received JointState

        # Preloaded home pose as default start state
        self.current_joint_state = JointState()
        self.current_joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        # UR5 ready position
        self.home_position = [
            0.0,
            -math.pi / 2,
            math.pi / 2,
            -math.pi / 2,
            -math.pi / 2,
            0.0
        ]
        self.current_joint_state.position = self.home_position

        # Subscribers, action and service clients
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_cb,
            10
        )
        self.act_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/ur5_arm_controller/follow_joint_trajectory"
        )
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")

        # Marker publisher with transient-local QoS
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, "waypoint_markers", qos)

    def joint_cb(self, msg):
        """Callback updates current JointState when data arrives."""
        try:
            self.current_joint_state = msg
            self.has_joint_state = True
        except Exception as e:
            self.get_logger().error(f"Error in JointState callback: {e}")

    def wait_for_joint_state(self, timeout=5.0):
        """Wait for first JointState or timeout."""
        start_time = time.time()
        self.get_logger().info("Waiting for JointState data...")
        while (
            rclpy.ok() and
            not self.has_joint_state and
            (time.time() - start_time) < timeout
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.has_joint_state

    def move_to_home(self):
        """Move the robot to the home position (fallback if no joint data)."""
        self.get_logger().info("Executing home commander fallback...")

        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.current_joint_state.name

        pt = JointTrajectoryPoint()
        pt.positions = self.home_position
        pt.time_from_start = Duration(seconds=3).to_msg()
        traj.points.append(pt)

        goal_msg.trajectory = traj

        self.get_logger().info("Sending home trajectory...")
        send_future = self.act_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.result():
            self.get_logger().error("Failed to send home goal!")
            return False

        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.result():
            self.get_logger().error("Home execution: result timeout")
            return False

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅ Home position reached")
            self.current_joint_state.position = self.home_position
            return True
        else:
            self.get_logger().error(f"❌ Home movement failed: code {result.error_code}")
            return False

    def wait_for_services(self):
        """Wait for action & IK/FK services to become available."""
        self.get_logger().info("Waiting for required services...")
        if not self.act_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK service not available!")
            return False
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("FK service not available!")
            return False
        return True

    def current_pose(self):
        """Compute current end-effector pose via FK."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["tool0"]
        req.robot_state.joint_state = self.current_joint_state
        req.header.frame_id = "world"
        req.header.stamp = self.get_clock().now().to_msg()
        try:
            future = self.fk_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            res = future.result()
            if not res or not res.pose_stamped:
                self.get_logger().error("FK request failed or timed out")
                return None
            return res.pose_stamped[0]
        except Exception as e:
            self.get_logger().error(f"FK error: {e}")
            return None

    def request_ik(self, pose: PoseStamped):
        """Request IK solution for a given pose."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur5_arm"
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout = Duration(seconds=2).to_msg()
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.joint_state = self.current_joint_state
        req.ik_request.robot_state.is_diff = True
        try:
            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            res = future.result()
            if not res or res.error_code.val != res.error_code.SUCCESS:
                self.get_logger().error(f"IK failed with code {res.error_code.val}")
                return None
            names = res.solution.joint_state.name
            poses = res.solution.joint_state.position
            return [
                poses[names.index(j)] if j in names else 0.0
                for j in self.current_joint_state.name
            ]
        except Exception as e:
            self.get_logger().error(f"IK error: {e}")
            return None

    def execute(self):
        """Main execution routine: interpolate Cartesian path and send trajectory."""
        if not self.wait_for_joint_state():
            self.get_logger().warn(
                "No JointState data received. Executing home fallback..."
            )
            if not self.move_to_home():
                self.get_logger().error("Home fallback failed. Aborting.")
                self.error_occurred = True
                return
            if not self.wait_for_joint_state(timeout=3.0):
                self.get_logger().warn(
                    "Still no live JointState after home. Using home pose."
                )
        else:
            self.get_logger().info("Received current JointState data.")

        if not self.wait_for_services():
            self.error_occurred = True
            return

        actual_start_pose = self.current_pose()
        if actual_start_pose:
            # Convert quaternion to Euler angles
            angles = euler_from_quaternion([
                actual_start_pose.pose.orientation.x,
                actual_start_pose.pose.orientation.y,
                actual_start_pose.pose.orientation.z,
                actual_start_pose.pose.orientation.w,
            ])
            start_xyzrpy = [
                actual_start_pose.pose.position.x,
                actual_start_pose.pose.position.y,
                actual_start_pose.pose.position.z,
                *angles
            ]
            self.get_logger().info(f"Start pose from FK: {start_xyzrpy}")
        else:
            start_xyzrpy = [0.0] * 6
            self.get_logger().warn("FK failed, using default start values.")

        # Build goal PoseStamped
        goal_vals = self.pose_xyzrpy
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "world"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z = goal_vals[:3]
        q = quaternion_from_euler(*goal_vals[3:])
        (
            goal_msg.pose.orientation.x,
            goal_msg.pose.orientation.y,
            goal_msg.pose.orientation.z,
            goal_msg.pose.orientation.w
        ) = q

        # Interpolate waypoints
        waypoints = interpolate_pose(start_xyzrpy, goal_vals, steps=50)

        marker_array = MarkerArray()
        traj = JointTrajectory()
        traj.joint_names = self.current_joint_state.name

        for i, wp in enumerate(waypoints):
            # Log each waypoint
            self.get_logger().info(
                f"Waypoint {i}: x={wp[0]:.3f}, y={wp[1]:.3f}, z={wp[2]:.3f}, "
                f"roll={wp[3]:.3f}, pitch={wp[4]:.3f}, yaw={wp[5]:.3f}"
            )

            # Create a visualization marker
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = wp[:3]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.02
            m.color.r = 1.0; m.color.a = 1.0
            m.lifetime.sec = 0
            marker_array.markers.append(m)

            # Compute IK for each waypoint
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = wp[:3]
            qwp = quaternion_from_euler(wp[3], wp[4], wp[5])
            (
                ps.pose.orientation.x,
                ps.pose.orientation.y,
                ps.pose.orientation.z,
                ps.pose.orientation.w
            ) = qwp

            joints = self.request_ik(ps)
            if joints is None:
                self.get_logger().error(f"IK failed at waypoint {i}")
                self.error_occurred = True
                return

            pt = JointTrajectoryPoint()
            pt.positions = joints
            pt.time_from_start = Duration(seconds=1.5 + i * 0.5).to_msg()
            traj.points.append(pt)

        # Publish markers
        self.get_logger().info(
            f"Publishing {len(marker_array.markers)} markers to /waypoint_markers"
        )
        self.marker_pub.publish(marker_array)

        # Send the trajectory
        goal_act = FollowJointTrajectory.Goal()
        goal_act.trajectory = traj
        self.get_logger().info("Sending trajectory to controller...")
        send_future = self.act_client.send_goal_async(goal_act)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Trajectory was rejected")
            self.execution_complete = True
            return
        self.get_logger().info("Trajectory accepted")
        handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        res = future.result().result
        if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Execution successful")
        else:
            self.get_logger().error(f"Execution failed with code {res.error_code}")
        self.execution_complete = True


def main():
    if len(sys.argv) != 7:
        print("Usage: ros2 run my_robot_control movelt_planning_obstacle <x> <y> <z> <roll> <pitch> <yaw>")
        return
    pose = [float(v) for v in sys.argv[1:]]
    os.environ["PYTHONUNBUFFERED"] = "1"
    rclpy.init()
    node = CartesianInterpUR5(pose)
    try:
        node.execute()
        start_time = time.time()
        while (
            rclpy.ok() and
            not node.execution_complete and
            not node.error_occurred and
            (time.time() - start_time) < 120
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.execution_complete and not node.error_occurred:
            node.get_logger().warn("Timeout during execution")
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted execution")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if node.error_occurred:
        sys.exit(1)


if __name__ == "__main__":
    main()
