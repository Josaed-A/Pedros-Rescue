"""
slam3d_pc.launch.py  --  PC (estacion de mando)

Consume los topicos que publica la Raspberry y corre el SLAM 3D:

    rgbd_odometry  -> estima la pose de la camara (odometria visual)
    rtabmap        -> construye el grafo y el mapa 3D
    rtabmap_viz    -> GUI

RTAB-Map NO corre en la Pi. rgbd_odometry extrayendo features a 30 Hz la satura
y da 2 Hz de odometria, que es inutilizable. La Pi solo publica; el PC procesa.

Uso:
    ros2 launch slam_3d slam3d_pc.launch.py

Por defecto borra ~/.ros/rtabmap.db al arrancar (--delete_db_on_start).
Para continuar un mapa existente, quita ese flag de rtabmap_args mas abajo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rtabmap_launch = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/depth_raw/image'),
        DeclareLaunchArgument(
            'camera_info_topic',
            # CRITICO: el camera_info del COLOR viene con k = [0,0,0,...] porque
            # la camara no esta calibrada y v4l2_camera no tiene de donde sacar
            # los intrinsecos. Sin fx/fy/cx/cy no se puede proyectar nada a 3D:
            # rtabmap extrae cientos de features y la odometria da quality=0
            # SIEMPRE. El driver OpenNI2 si rellena el camera_info de la
            # profundidad con valores reales (fx=fy=570.34, cx=319.5, cy=239.5).
            default_value='/depth_raw/camera_info',
        ),

        # --- TF: base_link -> camera ---
        # rpy = (-pi/2, 0, -pi/2) es la rotacion estandar entre la convencion de
        # robotica (X adelante, Z arriba) y la de vision (Z adelante, Y abajo).
        # Sin esto la nube aparece acostada.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera',
            ],
        ),

        # --- RTAB-Map ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'frame_id': 'base_link',

                # Color y profundidad vienen de DOS drivers independientes, sin
                # trigger comun, asi que nunca comparten timestamp exacto.
                'approx_sync': 'true',
                # 0.5 s de tolerancia. Sobre red los timestamps llegan mucho mas
                # dispersos que en local (donde 0.1 basta).
                'approx_sync_max_interval': '0.5',
                # Colas grandes por lo mismo. El default de 10 es muy corto
                # cuando el delay de red varia.
                'queue_size': '100',
                # Quita /rtabmap/odom_info de la sincronizacion: rtabmap pasa de
                # esperar 5 topicos alineados a esperar 4. Con 5 sobre red
                # practicamente nunca alinea y el nodo no procesa nada.
                'subscribe_odom_info': 'false',

                'rtabmap_args': (
                    '--delete_db_on_start '
                    '--Grid/DepthDecimation 2 '
                    '--Grid/CellSize 0.03 '
                    '--Rtabmap/DetectionRate 1 '
                    '--Odom/ImageDecimation 2'
                ),
            }.items(),
        ),
    ])
