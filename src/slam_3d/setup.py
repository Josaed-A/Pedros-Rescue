import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'slam_3d'


def redist_files():
    """Instala openni2_redist/ conservando la estructura de carpetas.

    Hace falta porque liborbbec.so tiene que quedar dentro de
    <redist>/OpenNI2/Drivers/ para que OpenNI2 lo encuentre.
    """
    out = []
    for root, _dirs, files in os.walk('openni2_redist'):
        if not files:
            continue
        dest = os.path.join('share', package_name, root)
        out.append((dest, [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ] + redist_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SabanaHerons',
    maintainer_email='semillero@unisabana.edu.co',
    description="SLAM 3D con Orbbec Astra Pro y RTAB-Map para Pedro's Rescue",
    license='MIT',
    entry_points={'console_scripts': []},
)
