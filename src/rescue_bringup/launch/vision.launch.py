"""
vision.launch.py
Launch standalone del módulo de visión: cámara + object_detector.

Soporta dos drivers:
  camera_driver:=astra_sdk  → driver oficial ros2_astra_camera (raw Image + CameraInfo)
                               Topics: /camera/color/image_raw, /camera/depth/image_raw
  camera_driver:=astra_core → driver del compañero astra_rgbd_camera_node (CompressedImage)
                               Topics: /robot/camera/astra/color|depth/image_raw/compressed

Uso:
  # Driver oficial (PC con SDK compilado)
  ros2 launch rescue_bringup vision.launch.py

  # Driver del compañero (Pi o sin SDK)
  ros2 launch rescue_bringup vision.launch.py camera_driver:=astra_core

  # Con modelo hazmat entrenado
  ros2 launch rescue_bringup vision.launch.py \\
    hazmat_model:=/workspace/src/rescue_bringup/models/hazmat_yolo.pt

  # Pi: driver compañero + sin YOLO COCO (más ligero) + modelo ONNX
  ros2 launch rescue_bringup vision.launch.py \\
    camera_driver:=astra_core enable_yolo:=false \\
    hazmat_model:=/workspace/src/rescue_bringup/models/hazmat_yolo.onnx
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rescue_bringup')

    hazmat_model   = LaunchConfiguration('hazmat_model',   default='')
    enable_yolo    = LaunchConfiguration('enable_yolo',    default='true')
    launch_rviz    = LaunchConfiguration('launch_rviz',    default='false')
    camera_driver  = LaunchConfiguration('camera_driver',  default='astra_sdk')

    # ── Driver oficial: ros2_astra_camera / orbbec_camera ────────────
    # Publica Image raw + CameraInfo en /camera/...
    camera_sdk_launch = TimerAction(
        period=0.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'camera.launch.py')
                ),
                launch_arguments={'driver': 'astra'}.items(),
                condition=LaunchConfigurationEquals('camera_driver', 'astra_sdk'),
            )
        ],
    )

    # ── Driver del compañero: astra_rgbd_camera_node ─────────────────
    # Publica CompressedImage en /robot/camera/astra/...
    camera_core_node = TimerAction(
        period=0.0,
        actions=[
            Node(
                package='rescue_robot_core',
                executable='astra_rgbd_camera_node',
                name='astra_rgbd_camera_node',
                output='screen',
                condition=LaunchConfigurationEquals('camera_driver', 'astra_core'),
                parameters=[{
                    'color_index':   2,
                    'depth_index':   -1,    # depth por OpenNI2 (si disponible)
                    'width':         640,
                    'height':        480,
                    'fps':           30,
                    'jpeg_quality':  80,
                    'mjpeg_passthrough': True,
                    'publish_point_cloud': False,
                }],
            )
        ],
    )

    # ── object_detector — configurado para cada driver ────────────────
    # astra_sdk: raw Image, topics /camera/...
    detector_sdk = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='object_detector',
                name='object_detector',
                output='screen',
                condition=LaunchConfigurationEquals('camera_driver', 'astra_sdk'),
                parameters=[{
                    'output_dir':          '/root/maps',
                    'team_name':           'PedrosRescue',
                    'mission':             'M1',
                    'robot_name':          'Pedro',
                    'mode':                'teleop',
                    'yolo_model':          'yolov8n.pt',
                    'hazmat_model':        hazmat_model,
                    'hazmat_conf':         0.40,
                    'enable_yolo':         enable_yolo,
                    'enable_apriltag':     True,
                    'enable_hazmat':       True,
                    'use_compressed':      False,
                    'color_topic':         '/camera/color/image_raw',
                    'depth_topic':         '/camera/depth/image_raw',
                    'camera_info_topic':   '/camera/color/camera_info',
                }],
            )
        ],
    )

    # astra_core: CompressedImage, topics /robot/camera/astra/...
    detector_core = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='object_detector',
                name='object_detector',
                output='screen',
                condition=LaunchConfigurationEquals('camera_driver', 'astra_core'),
                parameters=[{
                    'output_dir':          '/root/maps',
                    'team_name':           'PedrosRescue',
                    'mission':             'M1',
                    'robot_name':          'Pedro',
                    'mode':                'teleop',
                    'yolo_model':          'yolov8n.pt',
                    'hazmat_model':        hazmat_model,
                    'hazmat_conf':         0.40,
                    'enable_yolo':         enable_yolo,
                    'enable_apriltag':     True,
                    'enable_hazmat':       True,
                    'use_compressed':      True,
                    'color_topic':         '/robot/camera/astra/color/image_raw/compressed',
                    'depth_topic':         '/robot/camera/astra/depth/image_raw/compressed',
                    'camera_info_topic':   '/robot/camera/astra/camera_info',
                    # Intrínsecos Astra Pro (fallback cuando no hay CameraInfo)
                    'fx':                  525.0,
                    'fy':                  525.0,
                    'cx':                  319.5,
                    'cy':                  239.5,
                    'depth_scale':         0.001,
                }],
            )
        ],
    )

    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(launch_rviz),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_driver',
            default_value='astra_sdk',
            description='Driver de cámara: astra_sdk (oficial) | astra_core (compañero)',
        ),
        DeclareLaunchArgument(
            'hazmat_model',
            default_value='',
            description='Ruta al modelo hazmat .pt o .onnx. Vacío = usar detector HSV',
        ),
        DeclareLaunchArgument(
            'enable_yolo',
            default_value='true',
            description='Habilitar YOLO COCO (false = más ligero en Pi)',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Lanzar RViz2 para visualizar detecciones',
        ),
        camera_sdk_launch,
        camera_core_node,
        detector_sdk,
        detector_core,
        rviz_node,
    ])
