from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    params = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/config/farr_nav2_params.yaml'
    scan_params = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/config/farr_scan_from_mid360.yaml'
    urdf_path = '/root/farr_robot_ws/install/farr_bringup/share/farr_bringup/urdf/farr_base_laser.urdf'

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='farr_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_path]),
                'use_sim_time': False,
            }],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[scan_params],
            remappings=[('cloud_in', '/farr_slice_cloud'), ('scan', '/scan')],
        ),
        Node(
            package='farr_bringup',
            executable='odom_tf_broadcaster',
            name='farr_odom_tf_broadcaster',
            output='screen',
            parameters=[{'odom_topic': '/Odometry', 'source_frame': 'camera_init', 'odom_frame': 'odom', 'child_frame': 'base_link', 'sensor_frame': 'laser_link', 'output_odom_topic': '/farr_base_odom', 'publish_hz': 20.0, 'fastlio_body_to_laser_xyz': [0.0, 0.0, 0.0], 'fastlio_body_to_laser_rpy': [0.0, 0.0, 0.0]}],
        ),
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
                parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
            )],
        ),
    ])
