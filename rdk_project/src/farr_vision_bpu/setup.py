from setuptools import setup

package_name = 'farr_vision_bpu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FARR Team',
    maintainer_email='root@localhost',
    description='FARR Hikrobot camera publisher and RDK X5 BPU YOLO pose inference nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hik_camera_publisher = farr_vision_bpu.hik_camera_publisher:main',
            'person_pose_node = farr_vision_bpu.person_pose_node_official:main',
            'save_pose_result = farr_vision_bpu.save_pose_result:main',
        ],
    },
)
