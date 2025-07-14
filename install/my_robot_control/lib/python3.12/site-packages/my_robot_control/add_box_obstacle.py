#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__("obstacle_publisher")

        self.pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.timer = self.create_timer(1.0, self.publish_box)

    def publish_box(self):
        obj = CollisionObject()
        obj.header = Header()
        obj.header.frame_id = "world"
        obj.id = "test_box"

        # Box-Größe: 0.4 × 0.4 × 0.4
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.3, 0.3, 0.3]

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.2
        pose.pose.orientation.w = 1.0

        obj.primitives = [box]
        obj.primitive_poses = [pose.pose]
        obj.operation = CollisionObject.ADD

        self.pub.publish(obj)
        self.get_logger().info("Box veröffentlicht ✅")
        self.timer.cancel()  # nur einmal senden

def main():
    rclpy.init()
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
