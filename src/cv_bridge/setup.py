from setuptools import find_packages, setup


package_name = 'cv_bridge'

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
    description='Small Python CvBridge subset for Pedro Rescue image encodings.',
    license='Apache-2.0',
)
