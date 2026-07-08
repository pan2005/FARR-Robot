from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_static_guard = Node(
        package='farr_bringup',
        executable='imu_static_guard',
        name='farr_imu_static_guard',
        output='screen',
        parameters=[{
            'imu_topic': '/livox/imu',
            'window_sec': 5.0,
            'required_stable_windows': 2,
            'max_gyro_mean_abs': 0.05,
            'max_gyro_std': 0.12,
            'max_acc_norm_std': 0.35,
            'min_acc_norm_mean': 0.6,
            'max_acc_norm_mean': 1.4,
            'min_samples': 300,
            'max_wait_sec': 0.0,
            'log_period_sec': 1.0,
        }],
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('fast_lio'),
            'launch',
            'mapping.launch.py',
        ])),
        launch_arguments={'config_file': 'mid360.yaml', 'rviz': 'false'}.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('LD_PRELOAD', '/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0'),
        LogInfo(msg='FARR IMU static guard: keep the robot still; FAST-LIO starts after IMU is stable.'),
        imu_static_guard,
        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_static_guard,
                on_exit=[fast_lio_launch],
            )
        ),
    ])
