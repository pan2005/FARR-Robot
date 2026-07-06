from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('farr_bringup'), 'launch', name
        ]))
    )


def generate_launch_description():
    return LaunchDescription([
        include('sensors.launch.py'),
        TimerAction(period=3.0, actions=[include('slam.launch.py')]),
        TimerAction(period=5.0, actions=[include('vision.launch.py')]),
        TimerAction(period=8.0, actions=[include('web_gateway.launch.py')]),
    ])
