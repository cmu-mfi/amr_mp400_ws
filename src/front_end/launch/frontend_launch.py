from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    frontend_node = Node(
        package='front_end',
        executable='front_end',
        name='frontend_launch',
        output='screen'
    )

    return LaunchDescription([
        frontend_node
  ])