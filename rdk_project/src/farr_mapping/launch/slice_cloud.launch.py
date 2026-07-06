from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('z_min', default_value='-0.35'),
        DeclareLaunchArgument('z_max', default_value='0.85'),
        Node(
            package='farr_mapping',
            executable='slice_cloud',
            name='farr_slice_cloud',
            output='screen',
            parameters=[{
                'input_cloud_topic': '/cloud_registered',
                'output_cloud_topic': '/farr_slice_cloud',
                'z_min': LaunchConfiguration('z_min'),
                'z_max': LaunchConfiguration('z_max'),
                'process_every_n_clouds': 3,
                'max_points': 12000,
            }],
        ),
    ])
