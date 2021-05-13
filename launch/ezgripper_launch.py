from launch import LaunchDescription
from launch_ros.actions import Node
import os


params = os.path.join(
  '/home/rgoulart/workspaces/ros2_ws/src/ezgripper',
  'config',
  'params.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ezgripper',
            executable='ezgripper_node',
            name='ezgripper_node',
            parameters=[params],
            output='screen'
        ),

        Node(
            package='ezgripper',
            executable='ezgripper_node',
            name='ezgripper2_node',
            parameters=[params],
            output='screen'
        )
    ])