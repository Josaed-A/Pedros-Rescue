"""
test_hazmat_simple.launch.py
────────────────────────────
SOLO PRUEBA LOCAL — evalua el modelo hazmat con la arquitectura minima:

  una camara (logitech_pub, su propio OpenCV)
    → object_detector (camera_id=front, hazmat_mode=worker)
    → hazmat_worker (un solo modelo cargado)
    → hazmat_viewer (ventana OpenCV con el feed anotado)

Por defecto usa el modelo Edge Impulse (models/hazmat_ei_v3.eim). Para
comparar con el YOLO entrenado:
  ros2 launch rescue_bringup test_hazmat_simple.launch.py \
    hazmat_model:=$(ros2 pkg prefix rescue_bringup)/share/rescue_bringup/models/best.pt

AprilTag y YOLO-objetos quedan apagados para aislar el modelo hazmat
(enable_apriltag:=true / enable_yolo:=true los vuelven a encender).

La camara se autodetecta por nombre: GENERAL WEBCAM si esta conectada, si no
la camara integrada (HP HD Camera). Forzar con camera_device:=N.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Descubrimiento DDS 100% local (mismo motivo que en test_local_cameras.launch.py).
LOCAL_DDS_URI = (
    '<CycloneDDS><Domain><General>'
    '<AllowMulticast>true</AllowMulticast>'
    '</General></Domain></CycloneDDS>'
)

CAMERA_TOPIC = '/robot/camera/front/image_raw/compressed'
ANNOTATED_TOPIC = '/camera/color/image_annotated/compressed'


def _resolve_camera(name_hints, fallback: str) -> str:
    """Indice /dev/videoN de la primera camara cuyo nombre coincida (en orden
    de preferencia). Una camara expone varios nodos; el menor es el de captura."""
    names = {}
    for dev in glob.glob('/sys/class/video4linux/video*'):
        try:
            with open(os.path.join(dev, 'name')) as f:
                names[int(os.path.basename(dev)[5:])] = f.read().strip().lower()
        except (OSError, ValueError):
            continue
    for hint in name_hints:
        matches = [idx for idx, name in names.items() if hint.lower() in name]
        if matches:
            return str(min(matches))
    return fallback


def generate_launch_description():
    models_dir = os.path.join(get_package_share_directory('rescue_bringup'), 'models')
    default_camera = _resolve_camera(['GENERAL WEBCAM', 'HP HD Camera'], '0')

    camera_device = LaunchConfiguration('camera_device')
    hazmat_model = LaunchConfiguration('hazmat_model')
    hazmat_conf = LaunchConfiguration('hazmat_conf')

    camera_pub = Node(
        package='rescue_bringup', executable='logitech_pub', name='camera_pub',
        output='screen',
        parameters=[{'device': camera_device, 'topic': CAMERA_TOPIC,
                     'fps': 15, 'jpeg_quality': 80}],
    )

    hazmat_worker = TimerAction(period=1.0, actions=[Node(
        package='rescue_bringup', executable='hazmat_worker', name='hazmat_worker',
        output='screen',
        parameters=[{
            'camera_ids': ['front'],
            'hazmat_model': hazmat_model,
            'hazmat_conf': hazmat_conf,
            'hazmat_imgsz': 416,  # solo aplica a .pt
            'tick_period_sec': LaunchConfiguration('tick_period_sec'),
        }],
    )])

    detector = TimerAction(period=2.0, actions=[Node(
        package='rescue_bringup', executable='object_detector', name='object_detector_front',
        output='screen',
        parameters=[{
            'output_dir': os.path.join(os.path.expanduser('~'), 'maps'),
            'use_compressed': True,
            'color_topic': CAMERA_TOPIC,
            'require_depth': False,
            'camera_id': 'front',
            'enable_hazmat': True,
            'hazmat_mode': 'worker',
            'hazmat_conf': hazmat_conf,
            'enable_apriltag': LaunchConfiguration('enable_apriltag'),
            'enable_yolo': LaunchConfiguration('enable_yolo'),
            'alerts_dir': '',
            'detect_interval_sec': 0.1,
            'hazmat_submit_period_sec': LaunchConfiguration('hazmat_submit_period_sec'),
            'detection_hold_sec': 1.0,
        }],
    )])

    viewer = TimerAction(period=2.0, actions=[Node(
        package='rescue_bringup', executable='hazmat_viewer', name='hazmat_viewer',
        output='screen',
        parameters=[{'image_topic': ANNOTATED_TOPIC, 'result_topic': '/hazmat/result/front'}],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value=default_camera,
                              description='Indice /dev/videoN (autodetectado por nombre)'),
        DeclareLaunchArgument('hazmat_model',
                              default_value=os.path.join(models_dir, 'hazmat_ei_v3.eim'),
                              description='Modelo hazmat: .eim (Edge Impulse) o .pt (YOLO)'),
        DeclareLaunchArgument('hazmat_conf', default_value='0.40',
                              description='Confianza minima de una deteccion hazmat'),
        DeclareLaunchArgument('enable_apriltag', default_value='false'),
        DeclareLaunchArgument('enable_yolo', default_value='false'),
        DeclareLaunchArgument('hazmat_submit_period_sec', default_value='0.1',
                              description='Cada cuanto el detector envia un frame al worker'),
        DeclareLaunchArgument('tick_period_sec', default_value='0.05',
                              description='Periodo del scheduler del hazmat_worker'),
        SetEnvironmentVariable('CYCLONEDDS_URI', LOCAL_DDS_URI),
        SetEnvironmentVariable('ROS_DISCOVERY_SERVER', ''),
        camera_pub,
        hazmat_worker,
        detector,
        viewer,
    ])
