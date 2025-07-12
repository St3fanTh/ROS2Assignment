#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import sys
from math import pi
from geometry_msgs.msg import PoseStamped

from moveit.task_constructor import core, stages

def main(args=None):
    rclpy.init()

    # CLI-Argumente
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.4)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.3)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    cli_args = parser.parse_args()

    node = rclpy.create_node("reach_cli")
    task = core.Task()
    task.loadRobotModel(node)
    task.group = "panda_arm"
    task.name = "reaching_cli"

    # Aktuellen Zustand hinzufügen
    task.add(stages.CurrentState("Start"))

    # Zielpose definieren
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "world"
    goal_pose.pose.position.x = cli_args.x
    goal_pose.pose.position.y = cli_args.y
    goal_pose.pose.position.z = cli_args.z
    goal_pose.pose.orientation.w = 1.0  # keine Rotation (nur als Beispiel)

    move_to = stages.MoveTo("reach goal", core.PipelinePlanner())
    move_to.group = "panda_arm"
    move_to.setGoal(goal_pose)

    task.add(move_to)

    # Planung
    if task.plan():
        print("✅ Plan erfolgreich. Bewegung wird veröffentlicht.")
        task.publish(task.solutions[0])
    else:
        print("❌ Keine Lösung gefunden für die angegebene Pose.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
