from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_launch = Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'navigation_launch.py'
    params = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/config/farr_nav2_params.yaml'

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
            output='screen',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_launch)),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': params,
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
        Node(
            package='farr_chassis_bridge',
            executable='cmd_vel_bridge',
            name='farr_cmd_vel_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyS1',
                'baud': 115200,
                'send_hz': 50.0,
                'cmd_timeout': 0.12,
                'linear_scale': 1000.0,
                'angular_scale': 1000.0,
                'max_vx': 350.0,
                'max_vy': 250.0,
                'max_w': 450.0,
            }],
        ),
    ])
