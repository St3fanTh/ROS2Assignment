#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
import moveit_commander
import sys

class MoveItCartesianCommander(Node):
    def __init__(self):
        super().__init__('moveit_cartesian_commander')
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("panda_arm")  # oder dein PlanningGroup-Name

        self.group.set_pose_reference_frame("base_link")
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_planning_time(5.0)

    def send_pose_goal(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw

        self.group.set_pose_target(pose_goal)
        plan = self.group.plan()

        if plan[0]:
            self.get_logger().info("Plan erfolgreich, führe aus...")
            self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
        else:
            self.get_logger().warn("Kein Plan gefunden.")

def main():
    rclpy.init()
    commander = MoveItCartesianCommander()
    
    # Beispiel: bewege TCP zu (x=0.4, y=0.1, z=0.3)
    commander.send_pose_goal(0.4, 0.1, 0.3)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
