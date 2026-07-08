from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    elevation_launch = (
        Path(FindPackageShare('farr_mapping').find('farr_mapping')) /
        'launch' /
        'global_elevation_map.launch.py'
    )
    urdf_path = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/urdf/farr_base_laser.urdf'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='farr_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_path]),
            'use_sim_time': False,
        }],
    )

    odom_tf_node = Node(
        package='farr_bringup',
        executable='odom_tf_broadcaster',
        name='farr_odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/Odometry',
            'source_frame': 'camera_init',
            'odom_frame': 'odom',
            'child_frame': 'base_link',
            'sensor_frame': 'laser_link',
            'publish_hz': 20.0,
            'output_odom_topic': '/farr_base_odom',
            'fastlio_body_to_laser_xyz': [0.0, 0.0, 0.0],
            'fastlio_body_to_laser_rpy': [0.0, 0.0, 0.0],
        }],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        odom_tf_node,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(elevation_launch))),
    ])
