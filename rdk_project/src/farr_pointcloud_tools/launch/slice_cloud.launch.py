from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('z_min', default_value='0.05'),
        DeclareLaunchArgument('z_max', default_value='0.45'),
        DeclareLaunchArgument('mount_roll', default_value='0.0'),
        DeclareLaunchArgument('mount_pitch', default_value='0.0'),
        DeclareLaunchArgument('mount_yaw', default_value='0.0'),
        Node(
            package='farr_pointcloud_tools',
            executable='slice_cloud_node',
            name='farr_slice_cloud_cpp',
            output='screen',
            parameters=[{
                'input_cloud_topic': '/cloud_registered',
                'output_cloud_topic': '/farr_slice_cloud',
                'z_min': LaunchConfiguration('z_min'),
                'z_max': LaunchConfiguration('z_max'),
                'mount_roll': LaunchConfiguration('mount_roll'),
                'mount_pitch': LaunchConfiguration('mount_pitch'),
                'mount_yaw': LaunchConfiguration('mount_yaw'),
                'process_every_n_clouds': 1,
                'max_points': 30000,
            }],
        ),
    ])
