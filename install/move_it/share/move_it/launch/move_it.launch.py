from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='move_it',
            executable='cartesian_mover',
            output='screen',
            name='cartesian_mover',
            parameters=[{
                # You can add parameters here if needed
            }]
        )
    ])