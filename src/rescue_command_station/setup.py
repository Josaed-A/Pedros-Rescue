from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rescue_command_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedros rescue team',
    maintainer_email='pedros-rescue@example.com',
    description='PC command station for the Pedro Rescue robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ps4_teleop_node = rescue_command_station.nodes.ps4_teleop_node:main',
            'dashboard_node = rescue_command_station.nodes.dashboard_node:main',
            'rgbd_viewer_node = rescue_command_station.nodes.rgbd_viewer_node:main',
        ],
    },
)
