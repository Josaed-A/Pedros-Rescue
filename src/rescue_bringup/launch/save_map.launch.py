"""
save_map.launch.py
───────────────────
Guarda el mapa actual generado por slam_toolbox en disco.

Uso:
  ros2 launch rescue_bringup save_map.launch.py
  ros2 launch rescue_bringup save_map.launch.py map_name:=sala_robots map_dir:=/home/user/maps

El resultado son dos archivos:
  <map_name>.yaml   → metadatos del mapa (resolución, origen, etc.)
  <map_name>.pgm    → imagen del mapa en escala de grises

Para cargar el mapa después (localización):
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/ruta/al/mapa.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_name = LaunchConfiguration('map_name', default='rescue_map')
    map_dir  = LaunchConfiguration('map_dir',  default=os.path.expanduser('~/maps'))

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=[
            '-f', [map_dir, '/', map_name],
            '--ros-args',
            '-p', 'save_map_timeout:=5.0',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='rescue_map',
            description='Nombre del archivo del mapa (sin extensión)',
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value=os.path.expanduser('~/maps'),
            description='Directorio donde se guarda el mapa',
        ),
        map_saver,
    ])
