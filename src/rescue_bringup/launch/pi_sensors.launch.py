"""
pi_sensors.launch.py
─────────────────────
Stack de sensores para la Raspberry Pi 5 (headless, sin SLAM ni RViz).
El SLAM corre en el PC que recibe los topics via DDS.

Lanza:
  • robot_description  → TF tree (URDF)
  • ldlidar_node       → /ldlidar_node/scan
  • astra_camera_node  → /camera/depth/points
                         /camera/color/image_raw
                         /camera/depth_registered/points

Uso:
  ros2 launch rescue_bringup pi_sensors.launch.py
  ros2 launch rescue_bringup pi_sensors.launch.py launch_camera:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_camera = LaunchConfiguration('launch_camera', default='true')
    launch_lidar  = LaunchConfiguration('launch_lidar',  default='true')

    pkg_bringup = get_package_share_directory('rescue_bringup')

    # ── 1. TF tree (robot description) ─────────────────────────────
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'robot_description.launch.py')
        ),
    )

    # ── 2. LD19 lidar ───────────────────────────────────────────────
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'lidar_ld19.launch.py')
        ),
        condition=IfCondition(launch_lidar),
    )

    # ── 3. Orbbec Astra Pro (depth + color + point cloud) ──────────
    camera_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'camera.launch.py')
                ),
                launch_arguments={'driver': 'astra'}.items(),
                condition=IfCondition(launch_camera),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Lanzar cámara Orbbec Astra Pro (false = solo lidar)',
        ),
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Lanzar lidar LD19',
        ),
        robot_description_launch,
        lidar_launch,
        camera_launch,
    ])
