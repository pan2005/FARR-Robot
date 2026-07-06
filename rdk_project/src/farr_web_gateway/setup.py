from setuptools import setup

package_name = 'farr_web_gateway'

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
    maintainer_email='farr@example.com',
    description='FARR Web gateway for frontend HTTP/WebSocket protocol.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_gateway = farr_web_gateway.gateway_node:main',
        ],
    },
)
