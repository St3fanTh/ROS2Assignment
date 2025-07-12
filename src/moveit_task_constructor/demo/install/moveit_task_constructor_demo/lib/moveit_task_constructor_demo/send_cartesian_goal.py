#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose, PoseStamped

class CartesianMoveItNode(Node):
    def __init__(self):
        super().__init__('cartesian_moveit_node')
        
        # KEIN Namespace benötigt (alle Nodes im Root)
        self.moveit_py = MoveItPy(node_name="cartesian_moveit_node")
        
        # Planungsgruppe aus SRDF - typisch für Panda: "panda_arm"
        self.arm = self.moveit_py.get_planning_component("panda_arm")
        self.logger = self.get_logger()
        self.logger.info("MoveItPy initialisiert")

    def move_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # Frame-ID: Basis-Link des Panda (normalerweise "panda_link0")
        frame_id = "panda_link0"
        
        # Pose erstellen
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = qw  # Standardorientierung

        # PoseStamped mit korrektem Frame erstellen
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose

        # Ziel setzen
        self.arm.set_goal_state(pose_stamped_msg=pose_stamped)
        
        # Planung durchführen
        plan_result = self.arm.plan()
        
        # Plan ausführen
        if plan_result:
            self.logger.info("Plan erfolgreich - führe aus")
            robot_trajectory = plan_result.trajectory
            self.moveit_py.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planung fehlgeschlagen")

def main():
    rclpy.init()
    node = CartesianMoveItNode()
    
    # Beispiel-Pose: vor dem Roboter, etwas erhöht
    node.move_to_pose(0.3, 0.0, 0.5)  # x=0.3m, y=0.0m, z=0.5m
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()