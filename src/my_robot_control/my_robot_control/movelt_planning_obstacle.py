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
    """Lineare Interpolation zwischen zwei kartesischen Posen (6 DoF)"""
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
        self.has_joint_state = False  # Flag für empfangene Joint-Daten

        # Vorbelegte Home-Pose als Default-Startzustand
        self.current_joint_state = JointState()
        self.current_joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        # UR5-Ready Position
        self.home_position = [
            0.0,
            -math.pi/2,
            math.pi/2,
            -math.pi/2,
            -math.pi/2,
            0.0
        ]
        self.current_joint_state.position = self.home_position

        # Subscriber, Action- und Service-Clients
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

        # Marker-Publisher mit Transient-Local QoS
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', qos)

    def joint_cb(self, msg):
        """Callback aktualisiert aktuellen JointState, wenn Daten eintreffen"""
        try:
            # Direktes Update mit vollständiger Nachricht
            self.current_joint_state = msg
            self.has_joint_state = True  # Markiere als empfangen
        except Exception as e:
            self.get_logger().error(f"Fehler in JointState-Callback: {e}")

    def wait_for_joint_state(self, timeout=5.0):
        """Wartet auf ersten JointState oder Timeout"""
        start_time = time.time()
        self.get_logger().info("Warte auf JointState-Daten...")
        while rclpy.ok() and not self.has_joint_state and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.has_joint_state

    def move_to_home(self):
        """Bewegt den Roboter in die Home-Position (Fallback bei fehlenden Joint-Daten)"""
        self.get_logger().info("Führe Home-Commander aus...")
        
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.current_joint_state.name

        pt = JointTrajectoryPoint()
        pt.positions = self.home_position
        pt.time_from_start = Duration(seconds=3).to_msg()
        traj.points.append(pt)

        goal_msg.trajectory = traj

        self.get_logger().info("Sende Home-Trajektorie...")
        send_future = self.act_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.result():
            self.get_logger().error("Home-Goal konnte nicht gesendet werden!")
            return False

        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.result():
            self.get_logger().error("Home-Ausführung: Timeout auf Ergebnis")
            return False

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅ Home-Position erreicht")
            # Aktualisiere Joint-State auf Home-Position
            self.current_joint_state.position = self.home_position
            return True
        else:
            self.get_logger().error(f"❌ Fehler in Home-Bewegung: {result.error_code}")
            return False

    def wait_for_services(self):
        self.get_logger().info("Warte auf Services...")
        if not self.act_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action-Server nicht verfügbar!")
            return False
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK-Service nicht verfügbar!")
            return False
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("FK-Service nicht verfügbar!")
            return False
        return True

    def current_pose(self):
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
                self.get_logger().error("FK-Anfrage fehlgeschlagen oder timeout")
                return None
            return res.pose_stamped[0]
        except Exception as e:
            self.get_logger().error(f"FK-Fehler: {e}")
            return None

    def request_ik(self, pose):
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
                self.get_logger().error(f"IK-Fehler Code {res.error_code.val}")
                return None
            names = res.solution.joint_state.name
            poses = res.solution.joint_state.position
            return [poses[names.index(j)] if j in names else 0.0 for j in self.current_joint_state.name]
        except Exception as e:
            self.get_logger().error(f"IK-Fehler: {e}")
            return None

    def execute(self):
        # Warte auf Joint-Daten (mit 5s Timeout)
        if not self.wait_for_joint_state():
            self.get_logger().warn("Keine JointState-Daten erhalten. Führe Home-Commander aus...")
            if not self.move_to_home():
                self.get_logger().error("Home-Bewegung fehlgeschlagen. Abbruch.")
                self.error_occurred = True
                return
            # Nach Home-Bewegung erneut prüfen
            if not self.wait_for_joint_state(timeout=3.0):
                self.get_logger().warn("Trotz Home-Bewegung keine Live-Joint-Daten. Verwende Home-Pose.")
        else:
            self.get_logger().info("Aktuelle JointState-Daten erhalten")
            
        if not self.wait_for_services():
            self.error_occurred = True
            return

        # Versuche FK der aktuellen Pose
        actual_start = self.current_pose()
        if actual_start:
            # Konvertiere Quaternion zu Euler-Winkel
            angles = euler_from_quaternion([
                actual_start.pose.orientation.x,
                actual_start.pose.orientation.y,
                actual_start.pose.orientation.z,
                actual_start.pose.orientation.w,
            ])
            start_xyzrpy = [
                actual_start.pose.position.x,
                actual_start.pose.position.y,
                actual_start.pose.position.z,
                *angles
            ]
            self.get_logger().info(f"Startpose aus FK: {start_xyzrpy}")
        else:
            # Fallback auf Default-Home-Pose
            start_xyzrpy = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().warn("FK fehlgeschlagen, benutze voreingestellte Startwerte")

        # Zielpose bauen
        goal = self.pose_xyzrpy
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "world"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z = goal[:3]
        q = quaternion_from_euler(*goal[3:])
        goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z, goal_msg.pose.orientation.w = q

        # Interpolation
        waypoints = interpolate_pose(start_xyzrpy, goal, steps=50)

        marker_array = MarkerArray()
        traj = JointTrajectory()
        traj.joint_names = self.current_joint_state.name

        for i, wp in enumerate(waypoints):
            # Ausgabe des Wegpunkts in der Konsole
            self.get_logger().info(
                f"Wegpunkt {i}: x={wp[0]:.3f}, y={wp[1]:.3f}, z={wp[2]:.3f}, "
                f"roll={wp[3]:.3f}, pitch={wp[4]:.3f}, yaw={wp[5]:.3f}"
            )

            # Marker erzeugen
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = wp[0], wp[1], wp[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.02
            m.color.r = 1.0; m.color.a = 1.0
            m.lifetime.sec = 0
            marker_array.markers.append(m)

            # IK für jeden Wegpunkt
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = wp[:3]
            qwp = quaternion_from_euler(wp[3], wp[4], wp[5])
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qwp
            joints = self.request_ik(ps)
            if joints is None:
                self.get_logger().error(f"IK fehlgeschlagen bei WP {i}")
                self.error_occurred = True
                return
            pt = JointTrajectoryPoint()
            pt.positions = joints
            pt.time_from_start = Duration(seconds=1.5 + i*0.5).to_msg()
            traj.points.append(pt)

        # Marker publish
        self.get_logger().info(f"Veröffentliche {len(marker_array.markers)} Marker auf /waypoint_markers")
        self.marker_pub.publish(marker_array)

        # Trajektorie senden
        goal_act = FollowJointTrajectory.Goal()
        goal_act.trajectory = traj
        self.get_logger().info("Sende Trajektorie an Controller...")
        send_future = self.act_client.send_goal_async(goal_act)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Trajektorie abgelehnt")
            self.execution_complete = True
            return
        self.get_logger().info("Trajektorie akzeptiert")
        handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        res = future.result().result
        if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Ausführung erfolgreich")
        else:
            self.get_logger().error(f"Ausführung mit Fehlercode {res.error_code}")
        self.execution_complete = True


def main():
    if len(sys.argv) != 7:
        print("Usage: ros2 run my_robot_control movelt_planning_obstacle x y z roll pitch yaw")
        return
    pose = [float(v) for v in sys.argv[1:]]
    os.environ['PYTHONUNBUFFERED'] = '1'
    rclpy.init()
    node = CartesianInterpUR5(pose)
    try:
        node.execute()
        t0 = time.time()
        while rclpy.ok() and not node.execution_complete and not node.error_occurred and time.time() - t0 < 120:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.execution_complete and not node.error_occurred:
            node.get_logger().warn("Timeout während Ausführung")
    except KeyboardInterrupt:
        node.get_logger().info("User-Abbruch")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if node.error_occurred:
        sys.exit(1)


if __name__ == '__main__':
    main()
