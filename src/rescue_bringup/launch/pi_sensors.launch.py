"""
pi_sensors.launch.py
─────────────────────
Stack completo de sensores para la Raspberry Pi 5 (headless).
El SLAM corre en el PC y recibe los topics via DDS.

Lanza:
  1. robot_description   → TF tree (URDF)
  2. ldlidar_node        → /ldlidar_node/scan
  3. astra_camera_node   → /camera/depth/points
                           /camera/color/image_raw
  4. logitech_pub        → /robot/camera/front/image_raw/compressed
  5. object_detector     → /object_detections  /camera/color/image_annotated/compressed
  6. servos (brazo)      → /ax12a/*, /ex106/*, /joint_states  (opcional)

Uso:
  ros2 launch rescue_bringup pi_sensors.launch.py
  ros2 launch rescue_bringup pi_sensors.launch.py launch_logitech:=false
  ros2 launch rescue_bringup pi_sensors.launch.py launch_camera:=false
  ros2 launch rescue_bringup pi_sensors.launch.py launch_servos:=false
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
from launch_ros.actions import Node


def generate_launch_description():
    launch_camera   = LaunchConfiguration('launch_camera',   default='true')
    launch_lidar    = LaunchConfiguration('launch_lidar',    default='true')
    launch_logitech = LaunchConfiguration('launch_logitech', default='true')
    launch_servos   = LaunchConfiguration('launch_servos',   default='true')
    hazmat_model    = LaunchConfiguration('hazmat_model',    default='')

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

    # ── 3b. Relay Astra color → CompressedImage en /robot/camera/astra/ ──
    astra_republish = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='astra_relay',
                name='astra_color_relay',
                output='screen',
                parameters=[{'jpeg_quality': 70}],
                condition=IfCondition(launch_camera),
            )
        ],
    )

    # ── 4. Logitech frontal (detector corre en el PC) ─────────────
    logitech_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'logitech_vision.launch.py')
                ),
                launch_arguments={
                    'device':           '0',
                    'fps':              '30',
                    'hazmat_model':     hazmat_model,
                    'enable_yolo':      'true',
                    'launch_detector':  'false',
                }.items(),
                condition=IfCondition(launch_logitech),
            )
        ],
    )

    # ── 5. Servos del brazo 6-DOF (AX-12A + EX-106+) ─────────────
    servos_launch = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rescue_robot_core'), 'launch', 'servos.launch.py')),
            condition=IfCondition(launch_servos),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Lanzar cámara Orbbec Astra Pro',
        ),
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Lanzar lidar LD19',
        ),
        DeclareLaunchArgument(
            'launch_logitech',
            default_value='true',
            description='Lanzar cámara Logitech frontal + object_detector',
        ),
        DeclareLaunchArgument(
            'launch_servos',
            default_value='true',
            description='Lanzar drivers de servos del brazo 6-DOF (AX-12A + EX-106+)',
        ),
        DeclareLaunchArgument(
            'hazmat_model',
            default_value='',
            description='Ruta al modelo YOLO hazmat (.pt). Vacío = HSV fallback',
        ),
        robot_description_launch,
        lidar_launch,
        camera_launch,
        astra_republish,
        logitech_launch,
        servos_launch,
    ])
