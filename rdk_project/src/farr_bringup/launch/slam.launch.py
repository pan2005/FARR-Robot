from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('LD_PRELOAD', '/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0'),
        LogInfo(msg='FARR IMU static calibration: keep the robot still for 8 seconds before FAST-LIO starts.'),
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('fast_lio'),
                        'launch',
                        'mapping.launch.py',
                    ])),
                    launch_arguments={'config_file': 'mid360.yaml', 'rviz': 'false'}.items(),
                )
            ],
        ),
    ])
