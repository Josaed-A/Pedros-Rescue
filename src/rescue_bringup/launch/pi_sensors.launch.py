"""
pi_sensors.launch.py
─────────────────────
Stack completo de sensores para la Raspberry Pi 5 (headless).
El SLAM corre en el PC y recibe los topics via DDS.

Lanza:
  1. robot_description   → TF tree (URDF)
  2. ldlidar_node        → /ldlidar_node/scan
  3. astra_rgbd_camera_node → /robot/camera/astra/color/image_raw/compressed
                              /robot/camera/astra/depth/image_raw/compressed
                              /robot/camera/astra/points
  4. logitech_pub        → /robot/camera/front/image_raw/compressed
  5. object_detector     → /object_detections  /camera/color/image_annotated/compressed

Uso:
  ros2 launch rescue_bringup pi_sensors.launch.py
  ros2 launch rescue_bringup pi_sensors.launch.py launch_logitech:=false
  ros2 launch rescue_bringup pi_sensors.launch.py launch_camera:=false
  ros2 launch rescue_bringup pi_sensors.launch.py camera_driver:=astra_sdk
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_camera   = LaunchConfiguration('launch_camera',   default='true')
    launch_lidar    = LaunchConfiguration('launch_lidar',    default='true')
    launch_logitech = LaunchConfiguration('launch_logitech', default='true')
    hazmat_model    = LaunchConfiguration('hazmat_model',    default='')
    camera_driver   = LaunchConfiguration('camera_driver',   default='astra_core')
    astra_depth_index = LaunchConfiguration('astra_depth_index', default='-1')
    astra_color_index = LaunchConfiguration('astra_color_index', default='2')
    astra_fps = LaunchConfiguration('astra_fps', default='30')
    jpeg_quality = LaunchConfiguration('jpeg_quality', default='80')

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

    # ── 3a. Orbbec Astra Pro con driver propio del repo ───────────
    # Es el default porque no depende de submodulos externos.
    camera_core_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rescue_robot_core',
                executable='astra_rgbd_camera_node',
                name='astra_rgbd_camera_node',
                output='screen',
                condition=LaunchConfigurationEquals('camera_driver', 'astra_core'),
                parameters=[{
                    'depth_index': ParameterValue(astra_depth_index, value_type=int),
                    'color_index': ParameterValue(astra_color_index, value_type=int),
                    'fps': ParameterValue(astra_fps, value_type=int),
                    'jpeg_quality': ParameterValue(jpeg_quality, value_type=int),
                    'mjpeg_passthrough': True,
                    'publish_point_cloud': True,
                    'depth_frame_id': 'camera_optical_link',
                    'color_frame_id': 'camera_optical_link',
                }],
            )
        ],
        condition=IfCondition(launch_camera),
    )

    # ── 3b. Orbbec Astra Pro con driver oficial externo ───────────
    # Requiere tener astra_camera/orbbec_camera instalado o clonado aparte.
    camera_sdk_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'camera.launch.py')
                ),
                launch_arguments={'driver': 'astra'}.items(),
                condition=LaunchConfigurationEquals('camera_driver', 'astra_sdk'),
            )
        ],
        condition=IfCondition(launch_camera),
    )

    # ── 3c. Compatibilidad dashboard si se usa astra_sdk (/camera/...) ──
    astra_republish = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='astra_color_republish',
                output='screen',
                arguments=['raw', 'compressed'],
                remappings=[
                    ('in', '/camera/color/image_raw'),
                    ('out/compressed', '/robot/camera/astra/color/image_raw/compressed'),
                ],
                parameters=[{'jpeg_quality': 70}],
                condition=LaunchConfigurationEquals('camera_driver', 'astra_sdk'),
            ),
            Node(
                package='image_transport',
                executable='republish',
                name='astra_depth_republish',
                output='screen',
                arguments=['raw', 'compressed'],
                remappings=[
                    ('in', '/camera/depth/image_raw'),
                    ('out/compressed', '/robot/camera/astra/depth/image_raw/compressed'),
                ],
                parameters=[{'jpeg_quality': 70}],
                condition=LaunchConfigurationEquals('camera_driver', 'astra_sdk'),
            ),
        ],
        condition=IfCondition(launch_camera),
    )

    # ── 4+5. Logitech frontal + detector de objetos (YOLO + hazmat) ─
    logitech_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'logitech_vision.launch.py')
                ),
                launch_arguments={
                    'device':        '0',
                    'fps':           '15',
                    'hazmat_model':  hazmat_model,
                    'enable_yolo':   'true',
                }.items(),
                condition=IfCondition(launch_logitech),
            )
        ],
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
            'camera_driver',
            default_value='astra_core',
            description='Driver Astra: astra_core (propio) | astra_sdk (externo)',
        ),
        DeclareLaunchArgument('astra_depth_index', default_value='-1'),
        DeclareLaunchArgument('astra_color_index', default_value='2'),
        DeclareLaunchArgument('astra_fps', default_value='30'),
        DeclareLaunchArgument('jpeg_quality', default_value='80'),
        DeclareLaunchArgument(
            'hazmat_model',
            default_value='',
            description='Ruta al modelo YOLO hazmat (.pt). Vacío = HSV fallback',
        ),
        robot_description_launch,
        lidar_launch,
        camera_core_node,
        camera_sdk_launch,
        astra_republish,
        logitech_launch,
    ])
