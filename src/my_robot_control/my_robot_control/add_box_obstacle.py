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

        # Create a publisher for CollisionObject messages on the "/collision_object" topic
        self.pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        # Call publish_box() every second
        self.timer = self.create_timer(1.0, self.publish_box)

    def publish_box(self):
        # Create a new collision object
        obj = CollisionObject()
        obj.header = Header()
        obj.header.frame_id = "world"
        obj.id = "test_box"

        # Define a box primitive of size 0.3 × 0.3 × 0.3
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.3, 0.3, 0.3]

        # Specify the box pose in the 'world' frame
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.2
        pose.pose.orientation.w = 1.0

        # Attach the primitive and its pose to the collision object
        obj.primitives = [box]
        obj.primitive_poses = [pose.pose]
        obj.operation = CollisionObject.ADD

        # Publish the collision object
        self.pub.publish(obj)
        self.get_logger().info("Published box obstacle ✅")
        # Stop the timer so we only send once
        self.timer.cancel()

def main():
    rclpy.init()
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
