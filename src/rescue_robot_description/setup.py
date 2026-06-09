from glob import glob
from setuptools import find_packages, setup

package_name = 'rescue_robot_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rescue Team',
    maintainer_email='andresazgo@unisabana.edu.co',
    description='URDF/Xacro description for Pedro Rescue robot',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
