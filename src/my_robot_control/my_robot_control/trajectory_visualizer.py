#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetPositionFK
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # Service-Client für Forward Kinematics
        self.fk_cb_group = ReentrantCallbackGroup()
        self.fk_client = self.create_client(
            GetPositionFK, 
            '/compute_fk',
            callback_group=self.fk_cb_group
        )
        self.get_logger().info("Warte auf FK-Service...")
        while not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('FK-Service nicht verfügbar, erneut versuchen...')
        
        # Debug-Publisher für FK-Punkte
        self.debug_pub = self.create_publisher(PointStamped, '/debug_fk_points', 10)
        
        # Publisher für Visualisierungs-Marker
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        
        # Subscriber für Trajektorien-Nachrichten
        self.sub = self.create_subscription(
            JointTrajectory,
            '/arm_trajectory',
            self.trajectory_callback,
            10
        )
        self.get_logger().info("Trajectory Visualizer bereit...")
        
        # Testmarker senden
        self.publish_test_marker()

    def publish_test_marker(self):
        """Sende einen Testmarker zur Überprüfung der RViz-Verbindung"""
        marker_array = MarkerArray()
        
        # Testmarker (roter Würfel bei (0.5, 0, 0.5))
        test_marker = Marker()
        test_marker.header.frame_id = "world"
        test_marker.header.stamp = self.get_clock().now().to_msg()
        test_marker.ns = "test"
        test_marker.id = 100
        test_marker.type = Marker.CUBE
        test_marker.action = Marker.ADD
        test_marker.pose.position.x = 0.5
        test_marker.pose.position.y = 0.0
        test_marker.pose.position.z = 0.5
        test_marker.pose.orientation.w = 1.0
        test_marker.scale.x = 0.1
        test_marker.scale.y = 0.1
        test_marker.scale.z = 0.1
        test_marker.color.a = 1.0
        test_marker.color.r = 1.0
        test_marker.color.g = 0.0
        test_marker.color.b = 0.0
        marker_array.markers.append(test_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Testmarker gesendet (roter Würfel bei (0.5, 0, 0.5))")

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Neue Trajektorie mit {len(msg.points)} Punkten empfangen")
        
        if not msg.points:
            self.get_logger().warn("Leere Trajektorie erhalten!")
            return
            
        positions = []
        self.get_logger().info(f"Starte FK-Berechnung für {len(msg.points)} Punkte...")
        start_time = time.time()
        
        # FK für jeden Punkt der Trajektorie berechnen
        for i, point in enumerate(msg.points):
            # Bereite FK-Request vor
            req = GetPositionFK.Request()
            req.header.frame_id = "world"
            req.header.stamp = self.get_clock().now().to_msg()
            req.fk_link_names = ["panda_link8"]  # Endeffektor-Link
            
            # Setze Gelenkpositionen für diesen Punkt
            joint_state = JointState()
            joint_state.name = msg.joint_names
            joint_state.position = point.positions
            req.robot_state.joint_state = joint_state

            # FK anfragen
            future = self.fk_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            
            if res is None:
                self.get_logger().error(f"FK für Punkt {i} fehlgeschlagen: Keine Antwort")
                continue
                
            if res.error_code.val != res.error_code.SUCCESS:
                self.get_logger().error(f"FK-Fehler für Punkt {i}: Code {res.error_code.val}")
                continue
                
            if not res.pose_stamped:
                self.get_logger().error(f"Keine Pose in FK-Antwort für Punkt {i}")
                continue
                
            position = res.pose_stamped[0].pose.position
            positions.append(position)
            
            # Debug-Punkt veröffentlichen
            debug_point = PointStamped()
            debug_point.header.frame_id = "world"
            debug_point.header.stamp = self.get_clock().now().to_msg()
            debug_point.point = Point(x=position.x, y=position.y, z=position.z)
            self.debug_pub.publish(debug_point)
        
        duration = time.time() - start_time
        self.get_logger().info(f"FK-Berechnung abgeschlossen ({duration:.2f}s, {len(positions)} Punkte)")
        
        # Visualisiere die berechneten Positionen
        self.publish_markers(positions)

    def publish_markers(self, positions):
        if not positions:
            self.get_logger().warn("Keine Positionen zum Visualisieren!")
            return
            
        marker_array = MarkerArray()
        
        # Lösche vorherige Marker
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Marker für Trajektorien-Punkte
        points_marker = Marker()
        points_marker.header.frame_id = "world"
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "trajectory_points"
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.02
        points_marker.scale.y = 0.02
        points_marker.scale.z = 0.02
        points_marker.color.a = 1.0
        points_marker.color.r = 0.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0
        points_marker.points = [Point(x=p.x, y=p.y, z=p.z) for p in positions]
        marker_array.markers.append(points_marker)
        
        # Marker für Trajektorien-Linie
        line_marker = Marker()
        line_marker.header.frame_id = "world"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "trajectory_line"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.005
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.points = [Point(x=p.x, y=p.y, z=p.z) for p in positions]
        marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"{len(positions)} Punkte als Marker visualisiert")

def main():
    rclpy.init()
    visualizer = TrajectoryVisualizer()
    
    # Multi-Threaded Executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(visualizer)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()