from glob import glob
from setuptools import setup

package_name = 'farr_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FARR Team',
    maintainer_email='root@localhost',
    description='FARR launch and runtime configuration package.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = farr_bringup.odom_tf_broadcaster:main',
        ],
    },
)

