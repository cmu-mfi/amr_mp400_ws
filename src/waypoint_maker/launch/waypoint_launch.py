from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    waypointsrv_node = Node(
        package='waypoint_maker',
        executable='waypoint_service',
        name='waypoing_srv'
    )

    transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_node',
        arguments=[
            '0.0', '0.0', '0.3',  # X, Y, Z translation in meters
            '0.0', '0.0', '0.0',  # Roll, Pitch, Yaw rotation in radians
            'map',              # Parent frame ID
            'robot1/base_link'           # Child frame ID
        ]
    )

    return LaunchDescription([
        transform_node,
        waypointsrv_node,
  ])