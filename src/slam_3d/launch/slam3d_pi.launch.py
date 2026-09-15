"""
slam3d_pi.launch.py  --  RASPBERRY PI

Levanta los dos drivers de la Orbbec Astra Pro y los deja publicando:

    /depth_raw/image         16UC1 (milimetros)  ~30 Hz
    /depth_raw/camera_info   intrinsecos REALES del dispositivo
    /color/image_raw         rgb8                ~30 Hz

OJO: la Astra Pro son DOS dispositivos USB distintos:
    2bc5:0403 -> profundidad, solo habla OpenNI2
    2bc5:0501 -> color, es una camara UVC normal
Por eso hacen falta dos drivers separados.

Uso:
    ros2 launch slam_3d slam3d_pi.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Ruta por defecto del redist de OpenNI2 que trae liborbbec.so.
# Se puede sobreescribir con el argumento openni2_redist:=...
DEFAULT_REDIST = os.path.join(
    get_package_share_directory('slam_3d'),
    'openni2_redist',
    'arm64',
)


def generate_launch_description():
    openni2_redist = LaunchConfiguration('openni2_redist')
    color_device = LaunchConfiguration('color_device')

    # LD_PRELOAD es OBLIGATORIO.
    # El liborbbec.so de Orbbec esta compilado contra SU libOpenNI2 (2.3.x).
    # La libOpenNI2 que trae Ubuntu (2.2.0.33) es incompatible: carga el driver,
    # falla en silencio y reporta "Found 0 devices".
    # Forzando la libOpenNI2 de Orbbec el dispositivo aparece.
    openni_env = {
        'LD_PRELOAD': [openni2_redist, '/libOpenNI2.so'],
        'LD_LIBRARY_PATH': [openni2_redist, ':', os.environ.get('LD_LIBRARY_PATH', '')],
        'OPENNI2_DRIVERS_PATH': [openni2_redist, '/OpenNI2/Drivers'],
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'openni2_redist',
            default_value=DEFAULT_REDIST,
            description='Carpeta del redist de OpenNI2 de Orbbec (contiene libOpenNI2.so y OpenNI2/Drivers/liborbbec.so)',
        ),
        DeclareLaunchArgument(
            'color_device',
            default_value='/dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0',
            description='Path estable de la camara de color. NO usar /dev/videoN: el numero cambia entre maquinas y entre reconexiones.',
        ),
        # --- Profundidad (OpenNI2) ---
        Node(
            package='openni2_camera',
            executable='openni2_camera_driver',
            name='openni2_camera',
            output='screen',
            additional_env=openni_env,
            parameters=[{
                # Los dos frames tienen que llamarse igual que el child del TF
                # que publica slam3d_pc.launch.py, si no rtabmap no encuentra
                # la transformada y no arranca.
                'depth_frame_id': 'camera',
                'rgb_frame_id': 'camera',
            }],
        ),

        # --- Color (UVC / V4L2) ---
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            namespace='color',
            output='screen',
            parameters=[{
                'video_device': color_device,
                # 640x480 FIJO. La profundidad sale en 640x480 y rtabmap exige
                # que el tamano del color sea divisor exacto del de profundidad.
                # Con 320x240 el color y el depth no cuadran y la odometria falla
                # con "RGB size modulo depth size is not 0".
                'image_size': [640, 480],
                'camera_frame_id': 'camera',
            }],
        ),
    ])
