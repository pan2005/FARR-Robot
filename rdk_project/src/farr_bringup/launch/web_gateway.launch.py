from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farr_web_gateway',
            executable='web_gateway',
            name='farr_web_gateway',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': 8080,
                'status_hz': 5.0,
                'pointcloud_hz': 3.0,
                'max_points': 5000,
                'video_topic': '/person_pose/result_image',
                'detections_topic': '/person_pose/detections',
                'vision_status_topic': '/person_pose/status',
                'pointcloud_topic': '/farr_obstacle_cloud',
                'fallback_pointcloud_topic': '/cloud_registered',
                'vision_stale_sec': 3.0,
            }],
        ),
    ])
