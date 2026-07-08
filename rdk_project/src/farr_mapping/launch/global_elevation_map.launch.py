from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('map_size_x', default_value='24.0'),
        DeclareLaunchArgument('map_size_y', default_value='24.0'),
        DeclareLaunchArgument('max_range', default_value='10.0'),
        Node(
            package='farr_mapping',
            executable='global_elevation_mapper',
            name='farr_global_elevation_mapper',
            output='screen',
            parameters=[{
                'input_cloud_topic': '/cloud_registered',
                'map_frame': 'odom',
                'elevation_cloud_topic': '/farr/global_elevation_cloud',
                'elevation_grid_topic': '/farr/global_elevation_grid',
                'stats_topic': '/farr/global_elevation_stats',
                'resolution': LaunchConfiguration('resolution'),
                'map_size_x': LaunchConfiguration('map_size_x'),
                'map_size_y': LaunchConfiguration('map_size_y'),
                'max_range': LaunchConfiguration('max_range'),
                'min_range': 0.35,
                'self_filter_enabled': True,
                'self_filter_front': 0.46,
                'self_filter_rear': 0.46,
                'self_filter_left': 0.38,
                'self_filter_right': 0.38,
                'ground_quantile': 0.95,
                'ground_candidate_quantile': 0.65,
                'ground_plane_refine_distance': 0.10,
                'ground_plane_min_points': 100,
                'ground_inlier_ratio_min': 0.45,
                'ground_residual_std_max': 0.06,
                'min_height_clip': -0.20,
                'max_height_clip': 0.80,
                'min_points_per_cell': 2,
                'process_every_n_clouds': 2,
                'publish_period': 1.0,
                'save_directory': '/root/farr_maps',
                'save_name': 'global_elevation_map',
            }],
        ),
    ])
