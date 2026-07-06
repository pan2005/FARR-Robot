from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    nav2_launch = Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'navigation_launch.py'
    params = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/config/farr_nav2_params.yaml'
    scan_params = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/config/farr_scan_from_mid360.yaml'
    urdf_path = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/urdf/farr_base_laser.urdf'

    obstacle_filter_node = Node(
        package='farr_mapping',
        executable='slice_mapper',
        name='farr_nav_obstacle_filter',
        output='screen',
        parameters=[{
            'input_cloud_topic': '/cloud_registered',
            'map_topic': '/farr_nav_debug_map',
            'obstacle_cloud_topic': '/farr_nav_obstacle_cloud',
            'map_frame': 'odom',
            'z_min': 0.05,
            'z_max': 0.45,
            'resolution': 0.05,
            'map_size_x': 24.0,
            'map_size_y': 24.0,
            'max_range': 10.0,
            'odom_topic': '/farr_base_odom',
            'self_filter_enabled': True,
            'self_filter_front': 0.46,
            'self_filter_rear': 0.46,
            'self_filter_left': 0.38,
            'self_filter_right': 0.38,
            'self_filter_corner_radius': 0.05,
            'inflation_radius': 0.20,
            'use_relative_height': True,
            'ground_quantile': 0.95,
            'vertical_axis_sign': 1.0,
            'fit_ground_plane': True,
            'ground_candidate_quantile': 0.65,
            'ground_plane_refine_distance': 0.10,
            'ground_plane_min_points': 100,
            'obstacle_min_height': 0.12,
            'obstacle_max_height': 0.45,
            'hit_threshold': 3,
            'publish_period': 2.0,
            'process_every_n_clouds': 2,
            'unknown_as_free': True,
            'save_directory': '/root/farr_maps',
            'save_name': 'farr_nav_debug_map',
            'mount_roll': 0.0,
            'mount_pitch': 0.0,
            'mount_yaw': 0.0,
        }],
    )

    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[scan_params],
        remappings=[
            ('cloud_in', '/farr_nav_obstacle_cloud'),
            ('scan', '/scan'),
        ],
    )

    odom_tf_node = Node(
        package='farr_bringup',
        executable='odom_tf_broadcaster',
        name='farr_odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/Odometry',
            'source_frame': 'camera_init', 'odom_frame': 'odom',
            'child_frame': 'base_link',
            'sensor_frame': 'laser_link',
            'output_odom_topic': '/farr_base_odom',
            'fastlio_body_to_laser_xyz': [0.0, 0.0, 0.0],
            'fastlio_body_to_laser_rpy': [0.0, 0.0, 0.0],
        }],
    )

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

    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params],
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }],
            )],
        ),
    ]

    navigation_nodes = TimerAction(
        period=16.0,
        actions=[
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
                    'send_hz': 20.0,
                    'cmd_timeout': 0.35,
                    'linear_scale': 3400.0,
                    'angular_scale': 3800.0,
                    'max_vx': 1400.0,
                    'max_vy': 1200.0,
                    'max_w': 1600.0,
                }],
            ),
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        odom_tf_node,
        obstacle_filter_node,
        scan_node,
        *localization_nodes,
        navigation_nodes,
    ])
