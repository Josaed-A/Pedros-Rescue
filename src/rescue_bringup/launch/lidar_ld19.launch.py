"""
lidar_ld19.launch.py
─────────────────────
Levanta el driver del LDRobot LD19.

El driver usa composable nodes (lifecycle) para mejor rendimiento.
Publica:
  /ldlidar_node/scan  (sensor_msgs/LaserScan)  →  frame: ldlidar_link

Prerequisitos de hardware:
  • LD19 conectado por USB
  • Puerto serie configurable con el arg serial_port (default: /dev/ttyUSB0)

Regla udev recomendada para fijar el nombre del dispositivo:
  Crear /etc/udev/rules.d/99-ldlidar.rules con:
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ldlidar"
  Luego usar: serial_port:=/dev/ldlidar
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ld19_config = os.path.join(
        get_package_share_directory('rescue_bringup'),
        'config',
        'ld19_params.yaml',
    )

    # ── Lifecycle manager para el ldlidar ─────────────────────────
    lc_mgr_config = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr.yaml',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='ldlidar_lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config],
    )

    # ── Container con el componente LDLidar ──────────────────────
    # El componente publica ~/scan → /ldlidar_node/scan
    ldlidar_container = ComposableNodeContainer(
        name='ldlidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='ldlidar_component',
                plugin='ldlidar::LdLidarComponent',
                name='ldlidar_node',
                namespace='',
                parameters=[ld19_config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar reloj de simulación',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Puerto serie del LDRobot LD19',
        ),
        ldlidar_container,
        lifecycle_manager,
    ])
