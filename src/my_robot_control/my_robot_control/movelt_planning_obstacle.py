#!/usr/bin/env python3
"""
Moves the UR5 arm to a Cartesian target pose (x y z roll pitch yaw) and computes/visualizes
 the trajectory with linear interpolation.

Usage:
    ros2 run my_robot_control movelt_planning_obstacle 0.4 0.4 0.2 0.5 1.57 0

• IK service:   /compute_ik   (MoveIt)
• FK service:   /compute_fk
• Controller:   /ur5_arm_controller/follow_joint_trajectory
• Joint names:  shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                wrist_1_joint, wrist_2_joint, wrist_3_joint
"""

import sys
import math
import time
import os
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from tf_transformations import quaternion_from_euler, euler_from_quaternion


def interpolate_pose(start: List[float], goal: List[float], steps: int) -> List[List[float]]:
    """Linear interpolation between two Cartesian poses (6 DoF)."""
    interpolated = []
    for i in range(steps):
        alpha = i / (steps - 1)
        interpolated.append([(1 - alpha) * s + alpha * g for s, g in zip(start, goal)])
    return interpolated


class CartesianTrajectoryCommander(Node):
    """
    Combines:
      - waiting for joint states (with fallback home)
      - IK and FK calls
      - Cartesian interpolation and joint trajectory execution
      - RViz visualization of waypoints
    """
    def __init__(self, pose_xyzrpy: List[float]) -> None:
        super().__init__('cartesian_commander_ur5_interp')
        self.pose_xyzrpy = pose_xyzrpy
        self.execution_complete = False
        self.error_occurred = False

        # Home pose fallback
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.home_position = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
        self.current_joint_state = JointState()
        self.current_joint_state.name = self.joint_names
        self.current_joint_state.position = list(self.home_position)
        self.has_joint_state = False

        # Subscriber
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Action & Service clients
        self.act_client = ActionClient(self, FollowJointTrajectory,
                                       '/ur5_arm_controller/follow_joint_trajectory')
        # NOTE: wait_for_services moved to init to save time on each call
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', qos)

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        # Wait once for services
        if not self.wait_for_services():
            self.error_occurred = True

    def joint_cb(self, msg: JointState) -> None:
        """Save incoming JointState and mark that we've received one."""
        self.current_joint_state = msg
        self.has_joint_state = True

    def wait_for_joint_state(self, timeout: float = 5.0, rate_hz: float =10.0) -> bool:
        """Wait for the first JointState message or fallback after timeout."""
        self.get_logger().info('Waiting for JointState data...')
        rate = self.create_rate(rate_hz)
        start = time.time()
        while rclpy.ok() and not self.has_joint_state and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()
        if not self.has_joint_state:
            self.get_logger().warn('No JointState received; using home pose fallback.')
        return self.has_joint_state

    def move_to_home(self) -> bool:
        """Send the arm to the predefined home pose."""
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory(joint_names=self.joint_names)
        pt = JointTrajectoryPoint(positions=self.home_position,
                                  time_from_start=Duration(seconds=3).to_msg())
        traj.points.append(pt)
        goal.trajectory = traj
        self.get_logger().info('Sending home trajectory...')
        send_fut = self.act_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error('Home goal rejected')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Home position reached')
            return True
        self.get_logger().error(f'Home move failed: {res.error_code}')
        return False

    def wait_for_services(self) -> bool:
        self.get_logger().info('Waiting for action and service servers...')
        ok = (self.act_client.wait_for_server(timeout_sec=5.0) and
              self.ik_client.wait_for_service(timeout_sec=5.0) and
              self.fk_client.wait_for_service(timeout_sec=5.0))
        if not ok:
            self.get_logger().error('Required servers unavailable')
        return ok

    def current_pose(self) -> Optional[PoseStamped]:
        req = GetPositionFK.Request()
        req.fk_link_names = ['tool0']
        req.robot_state.joint_state = self.current_joint_state
        req.header.frame_id = 'world'
        req.header.stamp = self.get_clock().now().to_msg()
        fut = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.pose_stamped:
            return res.pose_stamped[0]
        self.get_logger().error('FK request failed')
        return None

    def request_ik(self, pose: PoseStamped) -> Optional[List[float]]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur5_arm'
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout = Duration(seconds=2).to_msg()
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.joint_state = self.current_joint_state
        req.ik_request.robot_state.is_diff = True
        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.error_code.val == res.error_code.SUCCESS:
            names = res.solution.joint_state.name
            poses = res.solution.joint_state.position
            return [poses[names.index(j)] if j in names else 0.0
                    for j in self.joint_names]
        self.get_logger().error(f'IK failed: {getattr(res.error_code, "val", None)}')
        return None

    def execute(self) -> None:
        """Main execution: ensure joint state, compute FK start, interpolate and send."""
        self.wait_for_joint_state()          # fallback to home if none
        if not self.has_joint_state:
            if not self.move_to_home():       # move home then wait again
                self.error_occurred = True
                return

        if not self.current_pose():
            self.get_logger().warn('FK failed for start pose; using home values')
            start_xyzrpy = [0.0]*6
        else:
            ps = self.current_pose()
            quat = [ps.pose.orientation.x, ps.pose.orientation.y,
                    ps.pose.orientation.z, ps.pose.orientation.w]
            angles = euler_from_quaternion(quat)
            start_xyzrpy = [ps.pose.position.x, ps.pose.position.y,
                            ps.pose.position.z, *angles]
        self.get_logger().info(f'Start pose: {start_xyzrpy}')

        # build goal PoseStamped
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = self.get_clock().now().to_msg()
        xyz, rpy = self.pose_xyzrpy[:3], self.pose_xyzrpy[3:]
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = xyz
        q = quaternion_from_euler(*rpy)
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q

        waypoints = interpolate_pose(start_xyzrpy, self.pose_xyzrpy, steps=50)
        marker_array = MarkerArray()
        traj = JointTrajectory(joint_names=self.joint_names)

        for i, wp in enumerate(waypoints):
            # log waypoint
            self.get_logger().info(f'Waypoint {i}: {wp}')
            # visualize
            m = Marker(header=goal.header, ns='waypoints', id=i,
                       type=Marker.SPHERE, action=Marker.ADD)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = wp[:3]
            m.scale.x = m.scale.y = m.scale.z = 0.02
            m.color.r = 1.0; m.color.a = 1.0
            marker_array.markers.append(m)

            # IK
            ps = PoseStamped(header=goal.header)
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = wp[:3]
            qr = quaternion_from_euler(*wp[3:])
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qr
            joints = self.request_ik(ps)
            if not joints:
                self.error_occurred = True
                return
            pt = JointTrajectoryPoint(positions=joints,
                                      time_from_start=Duration(seconds=1.5 + i*0.5).to_msg())
            traj.points.append(pt)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} markers')

        goal_act = FollowJointTrajectory.Goal(trajectory=traj)
        self.get_logger().info('Sending trajectory...')
        fut = self.act_client.send_goal_async(goal_act)
        fut.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Trajectory rejected')
            self.execution_complete = True
            return
        self.get_logger().info('Trajectory accepted')
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Execution successful')
        else:
            self.get_logger().error(f'Execution failed: {result.error_code}')
        self.execution_complete = True


# NOTE: The extensive logic in main() is a workaround for a ROS2 service bug.
# Without this loop and conditional checks, the node would remain blocked after sending a request,
# even though the server was running. This ensures the node properly spins and handles timeouts.
def main():
    if len(sys.argv) != 7:
        print('Usage: ros2 run my_robot_control movelt_planning_obstacle <x> <y> <z> <roll> <pitch> <yaw>')
        return
    os.environ['PYTHONUNBUFFERED'] = '1'
    rclpy.init()
    pose = [float(v) for v in sys.argv[1:]]
    node = CartesianTrajectoryCommander(pose)
    try:
        node.execute()
        start = time.time()
        while (rclpy.ok() and not node.execution_complete and not node.error_occurred and (time.time()-start)<120):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.execution_complete and not node.error_occurred:
            node.get_logger().warn('Timeout during execution')
    except KeyboardInterrupt:
        node.get_logger().info('User interrupted')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if node.error_occurred:
        sys.exit(1)

if __name__ == '__main__':
    main()
