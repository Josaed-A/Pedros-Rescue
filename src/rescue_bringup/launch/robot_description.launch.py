"""
robot_description.launch.py
────────────────────────────
Publica la descripción URDF del robot y los TF estáticos del árbol cinemático.

Nodos que levanta:
  • robot_state_publisher  → TF base_link → ldlidar_link, camera_link, wheels
  • joint_state_publisher  → estado de las juntas (para visualización)
  • static_transform_publisher → odom → base_footprint (identidad, sin odometría real)
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ── Cargar y procesar URDF con xacro ──────────────────────────
    pkg_desc = get_package_share_directory('rescue_robot_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'rescue_robot.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # ── robot_state_publisher ─────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── joint_state_publisher (sin GUI) ──────────────────────────
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── TF estático: odom → base_footprint (identidad) ───────────
    # Sin encoders el robot siempre aparece en (0,0,0) en el frame odom.
    # slam_toolbox publica map → odom para compensar el movimiento real.
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar reloj de simulación (false para hardware real)',
        ),
        rsp_node,
        jsp_node,
        static_tf_odom,
    ])
