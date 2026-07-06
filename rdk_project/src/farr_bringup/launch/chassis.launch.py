from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farr_chassis_bridge',
            executable='keyboard_control',
            parameters=[{'port': '/dev/ttyS1', 'baud': 115200}],
            output='screen',
        )
    ])
