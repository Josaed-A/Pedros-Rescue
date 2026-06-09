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
        ],
    },
)
