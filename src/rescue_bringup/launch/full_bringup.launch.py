"""
full_bringup.launch.py
───────────────────────
Sistema completo de Pedro's Rescue:

  PC (este nodo):
    ├── robot_description (URDF + TFs)
    ├── slam_toolbox       (/scan → /map + TF)
    ├── ldlidar_node       (LD19 → /scan)
    └── rviz2              (visualización)

  Raspberry Pi (nodo separado, misma red ROS 2):
    ├── motor_driver_node  (/cmd_vel → GPIO)
    └── [joy_node]         (/dev/input/jsX → /joy)

  PC — control:
    ├── joy_node           (PS4 → /joy)
    └── ps4_teleop_node    (/joy → /cmd_vel)

Uso rápido:
  # Terminal 1 (PC) – SLAM + lidar + visualización
  ros2 launch rescue_bringup full_bringup.launch.py

  # Terminal 2 (PC) – Teleop PS4
  ros2 launch rescue_bringup full_bringup.launch.py teleop:=true

  # Guardar mapa cuando termines:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/mi_mapa
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port  = LaunchConfiguration('serial_port',  default='/dev/ttyUSB0')
    teleop       = LaunchConfiguration('teleop',       default='false')

    pkg_bringup = get_package_share_directory('rescue_bringup')

    # ── 1. SLAM (incluye robot_description + lidar + slam_toolbox + rviz2)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'serial_port':  serial_port,
        }.items(),
    )

    # ── 2. Teleop PS4 (opcional, solo si teleop:=true) ────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(teleop),
    )

    ps4_node = Node(
        package='rescue_pc_brain',
        executable='ps4_teleop_node',
        name='ps4_teleop_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(teleop),
    )

    dashboard_node = Node(
        package='rescue_pc_brain',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        condition=IfCondition(teleop),
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
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Activar teleop PS4 en este PC (true/false)',
        ),
        slam_launch,
        joy_node,
        ps4_node,
        dashboard_node,
    ])
