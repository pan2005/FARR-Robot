from glob import glob
from setuptools import setup

package_name = 'farr_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='FARR Team',
    maintainer_email='root@localhost',
    description='Lightweight 2.5D mapping utilities for FARR.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slice_mapper = farr_mapping.slice_mapper:main',
            'slice_cloud = farr_mapping.slice_cloud:main',
            'global_elevation_mapper = farr_mapping.global_elevation_mapper:main',
        ],
    },
)
