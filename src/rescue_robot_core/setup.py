from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rescue_robot_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedros rescue team',
    maintainer_email='pedros-rescue@example.com',
    description='Raspberry Pi robot core for the Pedro Rescue platform.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_driver_node = rescue_robot_core.nodes.motor_driver_node:main',
            'logitech_camera_node = rescue_robot_core.camera_drivers.logitech_camera_node:main',
            'astra_rgbd_camera_node = rescue_robot_core.camera_drivers.astra_rgbd_camera_node:main',
            # ── Brazo 6-DOF — drivers de servos (corren en la Raspberry Pi) ──
            'dynamixel_bus_node = rescue_robot_core.nodes.dynamixel_bus_node:main',
            'dynamixel_sim_node = rescue_robot_core.nodes.dynamixel_sim_node:main',
            'ex106_driver_node = rescue_robot_core.nodes.ex106_driver_node:main',
        ],
    },
)
