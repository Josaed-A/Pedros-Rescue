from setuptools import find_packages, setup


package_name = 'joy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedros rescue team',
    maintainer_email='pedros-rescue@example.com',
    description='Minimal Linux joystick driver publishing sensor_msgs/Joy.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joy_node = joy.joy_node:main',
        ],
    },
)
