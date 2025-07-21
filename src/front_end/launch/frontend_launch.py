from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='front_end',
            executable='waypoint_gui',
            name='frontend_launch',
        ),
  ])