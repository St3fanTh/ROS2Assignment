import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
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
        self.current_joint_state = None
        self.execution_complete = False
        self.error_occurred = False

        # WORKAROUND: Manuelle Erstellung einer leeren JointState-Nachricht
        self.current_joint_state = JointState()
        self.current_joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.current_joint_state.position = [0.0] * 6

        # Services und Action Clients erstellen
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.act_client = ActionClient(
            self, FollowJointTrajectory, "/ur5_arm_controller/follow_joint_trajectory")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")

    def joint_cb(self, msg):
        """JointState-Callback mit Fehlerbehandlung"""
        try:
            # WORKAROUND: Manuelles Kopieren der Werte statt direkter Zuweisung
            self.current_joint_state.header = msg.header
            self.current_joint_state.name = msg.name
            self.current_joint_state.position = list(msg.position)
            self.current_joint_state.velocity = list(msg.velocity)
            self.current_joint_state.effort = list(msg.effort)
        except Exception as e:
            self.get_logger().error(f"Fehler in JointState-Callback: {str(e)}")

    def wait_for_services(self):
        """Warte auf benötigte Services und Action Server mit Timeout"""
        self.get_logger().info("Warte auf Services...")
        
        # Auf Action Server warten
        if not self.act_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action-Server nicht verfügbar!")
            return False
        
        # Auf IK-Service warten
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK-Service nicht verfügbar!")
            return False
            
        # Auf FK-Service warten
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("FK-Service nicht verfügbar!")
            return False
            
        return True

    def current_pose(self):
        """Aktuelle Pose via FK berechnen mit verbesserter Fehlerbehandlung"""
        req = GetPositionFK.Request()
        req.fk_link_names = ["tool0"]
        req.robot_state.joint_state = self.current_joint_state
        req.header.frame_id = "world"
        req.header.stamp = self.get_clock().now().to_msg()

        try:
            future = self.fk_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is None:
                self.get_logger().error("FK-Anfrage timeout!")
                return None
                
            res = future.result()
            
            if not res or not res.pose_stamped:
                self.get_logger().error("FK-Anfrage fehlgeschlagen!")
                return None
                
            return res.pose_stamped[0]
            
        except Exception as e:
            self.get_logger().error(f"FK-Fehler: {str(e)}")
            return None

    def request_ik(self, pose):
        """IK-Anfrage mit verbesserter Fehlerbehandlung"""
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
            
            if future.result() is None:
                self.get_logger().error("IK-Anfrage timeout!")
                return None
                
            res = future.result()
            
            if not res or res.error_code.val != res.error_code.SUCCESS:
                self.get_logger().error(f"IK-Fehler: Code {res.error_code.val}")
                return None

            joint_names = [
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
                
            # Extrahiere Gelenkpositionen sicher
            ik_dict = {}
            for i, name in enumerate(res.solution.joint_state.name):
                ik_dict[name] = res.solution.joint_state.position[i]
                
            return [ik_dict.get(name, 0.0) for name in joint_names]
            
        except Exception as e:
            self.get_logger().error(f"IK-Fehler: {str(e)}")
            return None

    def execute(self):
        """Hauptausführungslogik"""
        if not self.wait_for_services():
            self.error_occurred = True
            return

        # WORKAROUND: Kurze Verzögerung für JointState-Initialisierung
        start_time = time.time()
        while time.time() - start_time < 3.0:
            if any(p != 0.0 for p in self.current_joint_state.position):
                break
            self.get_logger().info("Warte auf JointState-Daten...", throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error("Keine JointState-Daten empfangen!")
            self.error_occurred = True
            return

        # Zielpose vorbereiten
        goal_pose = self.pose_xyzrpy
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = "world"
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_pose[0]
        goal_pose_msg.pose.position.y = goal_pose[1]
        goal_pose_msg.pose.position.z = goal_pose[2]
        q = quaternion_from_euler(goal_pose[3], goal_pose[4], goal_pose[5])
        goal_pose_msg.pose.orientation.x = q[0]
        goal_pose_msg.pose.orientation.y = q[1]
        goal_pose_msg.pose.orientation.z = q[2]
        goal_pose_msg.pose.orientation.w = q[3]

        # Startpose berechnen
        start_pose_msg = self.current_pose()
        if start_pose_msg is None:
            self.error_occurred = True
            return

        start_xyzrpy = [
            start_pose_msg.pose.position.x,
            start_pose_msg.pose.position.y,
            start_pose_msg.pose.position.z,
            *euler_from_quaternion([
                start_pose_msg.pose.orientation.x,
                start_pose_msg.pose.orientation.y,
                start_pose_msg.pose.orientation.z,
                start_pose_msg.pose.orientation.w,
            ])
        ]

        waypoints = interpolate_pose(start_xyzrpy, goal_pose, steps=50)

        # Trajektorie aufbauen
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        for i, wp in enumerate(waypoints):
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = wp[0]
            ps.pose.position.y = wp[1]
            ps.pose.position.z = wp[2]
            q = quaternion_from_euler(wp[3], wp[4], wp[5])
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q

            joint_positions = self.request_ik(ps)
            if joint_positions is None:
                self.get_logger().error(f"IK-Fehler bei Wegpunkt {i}. Abbruch.")
                self.error_occurred = True
                return

            self.get_logger().info(
                f"WP {i:02d} | x={wp[0]:.3f} y={wp[1]:.3f} z={wp[2]:.3f} | joints="
                + ", ".join(f"{p:+.2f}" for p in joint_positions)
            )

            pt = JointTrajectoryPoint()
            pt.positions = joint_positions
            pt.time_from_start = Duration(seconds=1.5 + i * 0.5).to_msg()
            traj.points.append(pt)

        # Action senden
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info("Sende Trajektorie an Controller...")
        self.send_goal_future = self.act_client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajektorie abgelehnt!")
            self.error_occurred = True
            self.execution_complete = True
            return

        self.get_logger().info("Trajektorie akzeptiert, warte auf Ergebnis...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Ausführung abgeschlossen")
        else:
            self.get_logger().error(f"Fehlercode: {result.error_code}")
        self.execution_complete = True


def main(argv=sys.argv[1:]):
    if len(argv) != 6:
        print("Aufruf: ros2 run my_robot_control movelt_planning_obstacle <x> <y> <z> <roll> <pitch> <yaw>")
        return

    pose_vals = [float(v) for v in argv]
    
    # WORKAROUND: Setze PYTHONUNBUFFERED Umgebungsvariable
    os.environ["PYTHONUNBUFFERED"] = "1"
    
    rclpy.init()
    node = CartesianInterpUR5(pose_vals)
    
    try:
        node.execute()
        
        # Hauptloop mit Timeout
        start_time = time.time()
        while (rclpy.ok() and 
               not node.execution_complete and 
               not node.error_occurred and 
               time.time() - start_time < 120.0):  # 2 Minuten Timeout
            rclpy.spin_once(node, timeout_sec=0.1)
            
        if not node.execution_complete and not node.error_occurred:
            node.get_logger().warn("Timeout während der Ausführung")
            
    except KeyboardInterrupt:
        node.get_logger().info("Abgebrochen durch Benutzer")
    except Exception as e:
        node.get_logger().error(f"Kritischer Fehler: {str(e)}")
    finally:
        # WORKAROUND: Explizites Zerstören des Nodes und Shutdown
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except:
            pass
        # Zusätzliche Bereinigung
        time.sleep(0.5)
        
    if node.error_occurred:
        sys.exit(1)


if __name__ == "__main__":
    main()