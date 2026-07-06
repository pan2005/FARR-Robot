from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    mapping_launch = Path(FindPackageShare('farr_mapping').find('farr_mapping')) / 'launch' / 'mapping_2_5d.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(mapping_launch))),
    ])
