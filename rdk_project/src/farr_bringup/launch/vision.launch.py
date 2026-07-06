from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('farr_bringup'), 'config', 'rdk_pose_runtime.yaml'])
    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', [
            TextSubstitution(text='/opt/MVS/lib/aarch64:'),
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ]),
        SetEnvironmentVariable('PYTHONPATH', [
            TextSubstitution(text='/opt/MVS/Samples/aarch64/Python/MvImport:'),
            EnvironmentVariable('PYTHONPATH', default_value=''),
        ]),
        Node(package='farr_vision_bpu', executable='hik_camera_publisher', output='screen'),
        Node(package='farr_vision_bpu', executable='person_pose_node', arguments=['--config', config], output='screen'),
    ])
