from setuptools import setup

package_name = 'farr_chassis_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='FARR Team',
    maintainer_email='farr@example.com',
    description='FARR chassis bridge between RDK X5 and STM32 over UART.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_control = farr_chassis_bridge.keyboard_control_node:main',
            'cmd_vel_bridge = farr_chassis_bridge.cmd_vel_bridge_node:main',
        ],
    },
)
