"""
camera.launch.py
────────────────
Lanza la cámara Orbbec Astra Pro para Pedro's Rescue.

Estrategia multi-driver:
  • v4l2    (default) RGB UVC /dev/video2 — siempre funciona
  • astra   ros2_astra_camera (SDK v1) — depth + RGB para Astra Pro original
  • openni2 openni2_camera + v4l2_camera por separado (fallback depth)
  • orbbec  OrbbecSDK v2 — para Astra 2 / Gemini (NO soporta Astra Pro original)

Parámetro:
  driver  (default: v4l2)  →  'v4l2' | 'astra' | 'openni2' | 'orbbec'
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_orbbec(context, *args, **kwargs):
    """Driver OrbbecSDK v2 — mejor para modelos Astra 2 / Gemini."""
    params = {
        # ── Streams activos ──────────────────────────────────────
        'enable_color': True,
        'enable_depth': True,
        'enable_left_ir': False,
        'enable_right_ir': False,
        # ── Resolución color ─────────────────────────────────────
        'color_width':  640,
        'color_height': 480,
        'color_fps':    30,
        'color_format': 'RGB888',
        # ── Resolución depth ─────────────────────────────────────
        'depth_width':  640,
        'depth_height': 480,
        'depth_fps':    30,
        # ── Alineación depth-color + nube 3D ─────────────────────
        'depth_registration':          True,
        'enable_colored_point_cloud':  True,
        'enable_point_cloud':          True,
        # ── Frame IDs (coincidir con URDF) ────────────────────────
        'camera_link_frame_id': 'camera_link',
        'depth_frame_id':       'camera_depth_frame',
        'color_frame_id':       'camera_color_frame',
        # ── Misc ──────────────────────────────────────────────────
        'enable_laser': True,
        'enable_ldp':   False,
        'device_preset': 'Default',
        'enable_sync_output_accel_gyro': False,
    }

    return [
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='orbbec_camera',
            namespace='camera',
            output='screen',
            parameters=[params],
            remappings=[
                ('/camera/color/image_raw',            '/camera/color/image_raw'),
                ('/camera/depth/image_raw',            '/camera/depth/image_raw'),
                ('/camera/depth_registered/points',    '/camera/depth_registered/points'),
            ],
        )
    ]


def launch_openni2(context, *args, **kwargs):
    """
    Driver OpenNI2 (fallback para Astra Pro original PID 0x0403).
    openni2_camera → depth + IR
    v4l2_camera    → RGB desde /dev/video2 (UVC PID 0x0501)
    """
    openni_node = Node(
        package='openni2_camera',
        executable='openni2_camera_node',
        name='openni2_camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'depth_registration': True,
            'use_device_time':    False,
            'rgb_frame_id':       'camera_color_frame',
            'depth_frame_id':     'camera_depth_frame',
            'rgb_camera_info_url':   '',
            'depth_camera_info_url': '',
        }],
    )

    rgb_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='rgb_camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'image_size':   [640, 480],
            'camera_frame_id': 'camera_color_frame',
            'pixel_format': 'YUYV',
            'camera_name':  'astra_rgb',
        }],
        remappings=[
            ('/camera/image_raw',     '/camera/color/image_raw'),
            ('/camera/camera_info',   '/camera/color/camera_info'),
        ],
    )

    return [openni_node, rgb_node]


def launch_astra(context, *args, **kwargs):
    """
    ros2_astra_camera (SDK v1) — soporta Astra Pro original (PID 0x0403).
    Publica depth + color + /camera/depth_registered/points con color.
    Requiere que liborbbec.so esté en el OpenNI2/Drivers del workspace.
    """
    return [
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            namespace='camera',
            output='screen',
            additional_env={
                'OPENNI2_REDIST':       '/workspace/install/astra_camera/lib',
                'OPENNI2_DRIVERS_PATH': '/workspace/install/astra_camera/lib/OpenNI2/Drivers',
            },
            parameters=[{
                'camera_name':          'astra',
                'camera_link_frame_id': 'camera_link',
                'color_frame_id':       'camera_color_frame',
                'depth_frame_id':       'camera_depth_frame',
                'ir_frame_id':          'camera_ir_frame',
                # Streams
                'enable_color':         True,
                'enable_depth':         True,
                'enable_ir':            False,
                # Resolución
                'color_width':          640,
                'color_height':         480,
                'color_fps':            30,
                'depth_width':          640,
                'depth_height':         480,
                'depth_fps':            30,
                # Alineación depth-color
                'depth_registration':            True,
                'enable_point_cloud':            True,
                'enable_colored_point_cloud':    True,
                # Misc
                'use_uvc_camera':       False,
                'uvc_vendor_id':        0x2BC5,
                'uvc_product_id':       0x0501,
                'publish_tf':           True,
                'tf_publish_rate':      10.0,
            }],
        )
    ]


def generate_launch_description():
    driver_arg = DeclareLaunchArgument(
        'driver',
        default_value='v4l2',
        description='Driver de cámara: v4l2 | astra | openni2 | orbbec',
    )

    def select_driver(context, *args, **kwargs):
        driver = LaunchConfiguration('driver').perform(context)
        if driver == 'openni2':
            return launch_openni2(context)
        if driver == 'orbbec':
            return launch_orbbec(context)
        if driver == 'astra':
            return launch_astra(context)
        # Default: v4l2 — RGB UVC camera (/dev/video2), siempre funciona
        return [
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='rgb_camera',
                namespace='camera',
                output='screen',
                parameters=[{
                    'video_device': '/dev/video2',
                    'image_size':   [640, 480],
                    'camera_frame_id': 'camera_color_frame',
                    'pixel_format': 'YUYV',
                    'camera_name':  'astra_rgb',
                }],
                remappings=[
                    ('/camera/image_raw',   '/camera/color/image_raw'),
                    ('/camera/camera_info', '/camera/color/camera_info'),
                ],
            )
        ]

    return LaunchDescription([
        driver_arg,
        OpaqueFunction(function=select_driver),
    ])
