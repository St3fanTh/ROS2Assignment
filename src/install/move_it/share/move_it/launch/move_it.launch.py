#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit.planning import MoveItPy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class CartesianMover(Node):
    def __init__(self):
        super().__init__('cartesian_mover')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.moveit = MoveItPy(node_name="moveit_py_node")
        self.panda_arm = self.moveit.get_planning_component("panda_arm")
        self.get_logger().info("MoveItPy initialized")

    def move_to_cartesian_pose(self, target_pose: Pose):
        # Set the goal pose
        self.panda_arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="panda_link8")
        
        # Plan and execute
        plan_result = self.panda_arm.plan()
        if plan_result:
            self.get_logger().info("Planning succeeded. Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().error("Planning failed!")

    def get_target_pose(self, x, y, z):
        # Create pose in world frame
        target_pose = Pose()
        target_pose.position = Point(x=x, y=y, z=z)
        target_pose.orientation = Quaternion(
            x=1.0, y=0.0, z=0.0, w=0.0
        )  # Adjust orientation as needed
        
        # Transform to panda_link0 frame
        try:
            transform = self.tf_buffer.lookup_transform(
                "panda_link0",
                "world",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5))
            
            # Apply transform (implementation depends on your TF setup)
            # For simplicity, we assume target is already in panda_link0 frame
            return target_pose
        except TransformException as ex:
            self.get_logger().error(f"TF error: {ex}")
            return None

def main(args=None):
    rclpy.init(args=args)
    
    # Example target position (adjust values as needed)
    TARGET_POSITION = [0.3, 0.2, 0.5]  # x, y, z in meters
    
    mover = CartesianMover()
    target_pose = mover.get_target_pose(*TARGET_POSITION)
    
    if target_pose:
        mover.move_to_cartesian_pose(target_pose)
    
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()