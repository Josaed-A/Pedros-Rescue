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
  │   └── ldlidar_node  →  /ldlidar_node/scan            │
  ├──────────────────────────────────────────────────────┤
  │  slam_toolbox (lifecycle node)                       │
  │   + slam_lifecycle_manager (configure + activate)   │
  │   Suscribe:  /ldlidar_node/scan                      │
  │   Publica:   /map  |  TF: map → odom                │
  └──────────────────────────────────────────────────────┘

Uso:
  ros2 launch rescue_bringup slam.launch.py
  ros2 launch rescue_bringup slam.launch.py serial_port:=/dev/ldlidar
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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time      = LaunchConfiguration('use_sim_time',    default='false')
    serial_port       = LaunchConfiguration('serial_port',    default='/dev/ttyUSB0')
    launch_rviz       = LaunchConfiguration('launch_rviz',    default='true')
    launch_lidar      = LaunchConfiguration('launch_lidar',   default='true')
    launch_camera     = LaunchConfiguration('launch_camera',  default='true')
    launch_robot_description = LaunchConfiguration('launch_robot_description', default='true')
    launch_detector   = LaunchConfiguration('launch_detector', default='false')
    hazmat_model_path = LaunchConfiguration('hazmat_model',   default='')
    output_dir        = LaunchConfiguration('output_dir')

    pkg_bringup = get_package_share_directory('rescue_bringup')
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_toolbox_params.yaml')

    # ── 1. Descripción del robot + TF tree ───────────────────────
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_robot_description),
    )

    # ── 2. Driver LDRobot LD19 ────────────────────────────────────
    # launch_lidar:=false cuando el lidar ya corre en la Pi
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'lidar_ld19.launch.py')
        ),
        launch_arguments={
            'serial_port':  serial_port,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(launch_lidar),
    )

    # ── 3a. depthimage_to_laserscan — depth cámara → scan 2D frontal ─────────
    # Convierte la imagen de profundidad de la Astra a un LaserScan virtual
    # al mismo plano horizontal que el LiDAR. Cubre el frente del robot donde
    # el LiDAR trasero tiene ángulo muerto o zona de sombra.
    depth_to_scan_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depth_to_laserscan',
                output='screen',
                parameters=[{
                    'scan_height':  5,
                    'range_min':    0.10,
                    'range_max':    4.0,
                    'output_frame': 'camera_link',
                }],
                remappings=[
                    ('depth',             '/camera/depth/image_raw'),
                    ('depth_camera_info', '/camera/depth/camera_info'),
                    ('scan',              '/camera/scan'),
                ],
            )
        ],
    )

    # ── 3b. scan_merger — fusiona LiDAR (trasero) + cámara (frontal) ─────────
    # Transforma ambos scans a base_footprint y combina en /scan_merged.
    # Resultado: cobertura 360° cooperativa sin punto ciego frontal.
    scan_merger_node = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='rescue_bringup',
                executable='scan_merger',
                name='scan_merger',
                output='screen',
                parameters=[{
                    'lidar_topic':     '/ldlidar_node/scan',
                    'camera_topic':    '/camera/scan',
                    'output_topic':    '/scan_merged',
                    'target_frame':          'base_footprint',
                    'angle_min':             -3.14159,
                    'angle_max':              3.14159,
                    'angle_increment':        0.00873,
                    'range_min':              0.10,
                    'range_max':             12.0,
                    # Filtro brazo: ignorar cono 310°→0°→30° del LiDAR
                    # Solo se usan rayos en el arco 30°–310° (lados + atrás)
                    'lidar_valid_min_deg':   30.0,
                    'lidar_valid_max_deg':  310.0,
                }],
            )
        ],
    )

    # ── 3c. slam_toolbox ─────────────────────────────────────────────────────
    # Usa /scan_merged: LiDAR trasero + scan virtual de cámara frontal.
    # La configuración vive en config/slam_toolbox_params.yaml (fuente única de
    # verdad). Aquí solo se sobreescribe lo que depende del launch.
    # Con use_lifecycle_manager:=false el nodo async se auto-configura y
    # auto-activa (patrón estándar de slam_toolbox online_async), por lo que NO
    # se llama a `ros2 lifecycle set` manualmente.
    slam_node = TimerAction(
        period=6.5,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {
                        'use_sim_time': use_sim_time,
                        'scan_topic': '/scan_merged',
                    },
                ],
            )
        ],
    )

    # ── 5. Cámara Orbbec Astra Pro ────────────────────────────────
    # launch_camera:=false cuando las cámaras ya corren en la Pi
    camera_launch = TimerAction(
        period=4.0,
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

    # ── 6. pointcloud_accumulator — acumula nube 3D para PLY ─────
    pc_accum_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='pointcloud_accumulator',
                name='pointcloud_accumulator',
                output='screen',
                parameters=[{
                    'output_dir':  output_dir,
                    'team_name':   'SabanaHerons',
                    'mission':     'M1',
                    'voxel_size':  0.025,
                    'max_range':   4.0,
                    'min_range':   0.25,
                    'sample_rate': 3,
                }],
            )
        ],
    )

    # ── 7. geotiff_writer — trackea ruta y genera GeoTIFF ───────
    # Se inicia después del lifecycle manager. Llama al servicio
    # /save_geotiff para exportar el mapa compliant con RoboCup 2026.
    geotiff_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='geotiff_writer',
                name='geotiff_writer',
                output='screen',
                parameters=[{
                    'output_dir':  output_dir,
                    'team_name':   'SabanaHerons',
                    'mission':     'M1',
                    'path_step_m': 0.08,
                }],
            )
        ],
    )

    # ── 8. object_detector — detección AprilTag + hazmat + YOLO ────
    # Habilitar con: ros2 launch rescue_bringup slam.launch.py launch_detector:=true
    # El parámetro hazmat_model apunta al modelo entrenado; si está vacío usa detector HSV.
    detector_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='object_detector',
                name='object_detector',
                output='screen',
                condition=IfCondition(launch_detector),
                parameters=[{
                    'output_dir':    output_dir,
                    'team_name':     'SabanaHerons',
                    'country':       'Colombia',
                    'mission':       'M1',
                    'robot_name':    'Pedro',
                    'mode':          'T',
                    'yolo_model':    '/workspace/src/rescue_bringup/models/mission_objects_yolo.pt',
                    'hazmat_model':  hazmat_model_path,
                    'hazmat_conf':   0.65,
                    'enable_yolo':   True,
                    'enable_apriltag': True,
                    'enable_hazmat': True,
                }],
            )
        ],
    )

    # ── 9. RViz2 para visualización (solo si launch_rviz:=true) ─────
    rviz_config = os.path.join(pkg_bringup, 'config', 'slam_rviz.rviz')
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(launch_rviz),
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
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Lanzar RViz2 (false para Pi headless)',
        ),
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Lanzar driver lidar (false si ya corre en Pi)',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Lanzar driver cámara (false si ya corre en Pi)',
        ),
        DeclareLaunchArgument(
            'launch_robot_description',
            default_value='true',
            description='Publicar URDF/TF del robot desde este launch',
        ),
        DeclareLaunchArgument(
            'launch_detector',
            default_value='false',
            description='Lanzar object_detector (AprilTag + hazmat YOLO + objetos YOLO)',
        ),
        DeclareLaunchArgument(
            'hazmat_model',
            default_value='',
            description='Ruta al modelo hazmat entrenado (.pt). Vacío = usar detector HSV',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/workspace/maps',
            description='Directorio para GeoTIFF, PLY y CSV de mision',
        ),
        robot_description_launch,
        lidar_launch,
        camera_launch,
        depth_to_scan_node,
        scan_merger_node,
        slam_node,
        pc_accum_node,
        geotiff_node,
        detector_node,
        rviz_node,
    ])
