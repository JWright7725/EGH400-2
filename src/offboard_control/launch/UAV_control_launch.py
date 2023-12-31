from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control',
            namespace='UAV1',
            executable='UAV_control',
            name='sim',
            output="screen",
            emulate_tty=True,
            parameters=[
                {'UAV_ID': '1'}
            ]
        ),
        Node(
            package='offboard_control',
            namespace='UAV2',
            executable='UAV_control',
            name='sim',
            output="screen",
            emulate_tty=True,
            parameters=[
                {'UAV_ID': '2'}
            ]
        )
    ])