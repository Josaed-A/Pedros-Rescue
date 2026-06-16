from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_brazo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ax12a_driver     = control_brazo.ax12a_driver_node:main',
            'ex106_driver     = control_brazo.ex106_driver_node:main',
            'ax12a_sim_driver = control_brazo.sim_driver_node:main',
            'ex106_sim_driver = control_brazo.sim_driver_node:main',
            'cinematica       = control_brazo.cinematica_node:main',
            'simulacion       = control_brazo.simulacion_node:main',
            'gui_control      = control_brazo.gui_node:main',
        ],
    },
)
