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
  │   └── ldlidar_node  →  /scan                        │
  ├──────────────────────────────────────────────────────┤
  │  slam_toolbox (async_slam_toolbox_node)              │
  │   Suscribe:  /scan                                   │
  │   Publica:   /map  |  TF: map → odom                │
  └──────────────────────────────────────────────────────┘

Uso:
  ros2 launch rescue_bringup slam.launch.py
  ros2 launch rescue_bringup slam.launch.py serial_port:=/dev/ldlidar
  ros2 launch rescue_bringup slam.launch.py save_map_on_exit:=true map_name:=mi_mapa
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
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_toolbox_params.yaml')

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

    # ── 3. slam_toolbox ──────────────────────────────────────────
    # Se retrasa 2 s para asegurar que el TF tree ya está publicado
    slam_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {'use_sim_time': use_sim_time},
                ],
            )
        ],
    )

    # ── 4. RViz2 para visualización ──────────────────────────────
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
        rviz_node,
    ])
