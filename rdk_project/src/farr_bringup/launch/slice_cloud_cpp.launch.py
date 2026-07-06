from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    launch_file = Path(FindPackageShare('farr_pointcloud_tools').find('farr_pointcloud_tools')) / 'launch' / 'slice_cloud.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)))])
