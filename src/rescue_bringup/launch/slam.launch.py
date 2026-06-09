"""
slam.launch.py
───────────────
Stack completo de SLAM para Pedro's Rescue:

  ┌──────────────────────────────────────────────────────┐
  │  robot_description.launch.py                         │
  │   ├── robot_state_publisher  (URDF → TFs)            │
  │   ├── joint_state_publisher                          │
  │   └── static_tf: odom → base_footprint              │
  ├──────────────────────────────────────────────────────┤
  │  lidar_ld19.launch.py                                │
  │   └── ldlidar_node  →  /ldlidar_node/scan            │
  ├──────────────────────────────────────────────────────┤
  │  slam_toolbox (lifecycle node)                       │
  │   + slam_lifecycle_manager (configure + activate)   │
  │   Suscribe:  /ldlidar_node/scan                      │
  │   Publica:   /map  |  TF: map → odom                │
  └──────────────────────────────────────────────────────┘

Uso:
  ros2 launch rescue_bringup slam.launch.py
  ros2 launch rescue_bringup slam.launch.py serial_port:=/dev/ldlidar
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port  = LaunchConfiguration('serial_port',  default='/dev/ttyUSB0')

    pkg_bringup = get_package_share_directory('rescue_bringup')

    # ── 1. Descripción del robot + TF tree ───────────────────────
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 2. Driver LDRobot LD19 ────────────────────────────────────
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'lidar_ld19.launch.py')
        ),
        launch_arguments={
            'serial_port':  serial_port,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── 3. slam_toolbox (lifecycle node) ─────────────────────────
    # slam_toolbox es un LifecycleNode — necesita ser configurado y
    # activado externamente. use_lifecycle_manager: true indica que
    # esperará al slam_lifecycle_manager para transicionar.
    slam_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'scan_topic': '/ldlidar_node/scan',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_footprint',
                    'mode': 'mapping',
                    'use_map_saver': False,
                    'use_lifecycle_manager': True,
                    'debug_logging': False,
                    'resolution': 0.05,
                    'max_laser_range': 12.0,
                    'transform_timeout': 0.5,
                    'tf_buffer_duration': 30.0,
                    'minimum_travel_distance': 0.0,
                    'minimum_travel_heading': 0.0,
                    'map_update_interval': 1.0,
                    'throttle_scans': 1,
                    'correlation_search_space_dimension': 2.0,
                    'correlation_search_space_resolution': 0.01,
                    'correlation_search_space_smear_deviation': 0.1,
                    'stack_size_to_use': 40000000,
                }],
                remappings=[
                    ('/scan', '/ldlidar_node/scan'),
                ],
            )
        ],
    )

    # ── 4. Lifecycle manager para slam_toolbox ────────────────────
    # Se retrasa 1 s más que slam_toolbox para que el nodo ya esté
    # registrado. autostart: true → configura y activa automáticamente.
    slam_lifecycle_manager = TimerAction(
        period=6.5,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='slam_lifecycle_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                }],
            )
        ],
    )

    # ── 5. RViz2 para visualización ──────────────────────────────
    rviz_config = os.path.join(pkg_bringup, 'config', 'slam_rviz.rviz')
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            )
        ],
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
        robot_description_launch,
        lidar_launch,
        slam_node,
        slam_lifecycle_manager,
        rviz_node,
    ])
