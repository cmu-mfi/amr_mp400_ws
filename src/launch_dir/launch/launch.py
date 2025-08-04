from flask.cli import F
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    waypointsrv_node = Node(
        package='waypoint_maker',
        executable='waypoint_service',
        name='waypoing_srv'
    )

    predocker_node = Node(
        package='pre_docking',
        executable='pre_docking_srv',
        name='predocking_launch',
        output='screen'
    )

    frontend_node = Node(
        package='front_end',
        executable='front_end',
        name='frontend_launch',
        output='screen'
    )

    transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_node',
        arguments=[
            '0.0', '0.0', '0.3',  # X, Y, Z translation in meters
            '0.0', '0.0', '0.0',  # Roll, Pitch, Yaw rotation in radians
            'robot1/base_link',              # Parent frame ID
            'robot1_color_frame'              # Child frame ID
        ]
    )



    return LaunchDescription([
        frontend_node,
        waypointsrv_node,
        predocker_node,
        transform_node
  ])