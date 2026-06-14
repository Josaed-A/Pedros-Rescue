"""
logitech_vision.launch.py
Módulo de visión con cámara Logitech USB (sin profundidad).

Lanza:
  - logitech_pub   → publica CompressedImage en /robot/camera/front/image_raw/compressed
  - object_detector → detección AprilTag + YOLO COCO + hazmat YOLOv8

Uso:
  ros2 launch rescue_bringup logitech_vision.launch.py
  ros2 launch rescue_bringup logitech_vision.launch.py device:=0
  ros2 launch rescue_bringup logitech_vision.launch.py device:=2 hazmat_model:=/workspace/src/rescue_bringup/models/hazmat_yolo.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device       = LaunchConfiguration('device',       default='2')
    hazmat_model = LaunchConfiguration('hazmat_model', default='')
    enable_yolo  = LaunchConfiguration('enable_yolo',  default='true')
    fps          = LaunchConfiguration('fps',           default='15')

    camera_node = Node(
        package='rescue_bringup',
        executable='logitech_pub',
        name='logitech_pub',
        output='screen',
        parameters=[{
            'device':        device,
            'fps':           fps,
            'jpeg_quality':  80,
            'topic':         '/robot/camera/front/image_raw/compressed',
        }],
    )

    detector_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='object_detector',
                name='object_detector',
                output='screen',
                parameters=[{
                    'output_dir':        '/root/maps',
                    'team_name':         'PedrosRescue',
                    'mission':           'M1',
                    'robot_name':        'Pedro',
                    'mode':              'teleop',
                    # Visión
                    'use_compressed':    True,
                    'color_topic':       '/robot/camera/front/image_raw/compressed',
                    'require_depth':     False,
                    # Detección
                    'enable_apriltag':   True,
                    'enable_hazmat':     True,
                    'enable_yolo':       enable_yolo,
                    'yolo_model':        'yolov8n.pt',
                    'hazmat_model':      hazmat_model,
                    'hazmat_conf':       0.40,
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('device',       default_value='2',  description='Índice /dev/videoN de la Logitech'),
        DeclareLaunchArgument('hazmat_model', default_value='',   description='Ruta al .pt o .onnx. Vacío = HSV'),
        DeclareLaunchArgument('enable_yolo',  default_value='true', description='Habilitar YOLO COCO'),
        DeclareLaunchArgument('fps',          default_value='15', description='FPS de captura'),
        camera_node,
        detector_node,
    ])
