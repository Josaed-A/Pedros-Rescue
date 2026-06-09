from glob import glob
from setuptools import find_packages, setup

package_name = 'rescue_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config',  glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rescue Team',
    maintainer_email='andresazgo@unisabana.edu.co',
    description='Launch and config for Pedro Rescue SLAM stack',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
