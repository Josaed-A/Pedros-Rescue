# arm_pi.launch.py — lado Raspberry Pi del brazo 6-DOF para Pedro's Rescue.
#
# Lanza SOLO los drivers reales de los Dynamixel (AX-12A + EX-106+) y sus
# encoders/"contador". La cinematica, el control cartesiano y la GUI corren en
# el PC (ver arm_station.launch.py).
#
#   ros2 launch control_brazo arm_pi.launch.py
#
# Puertos y baudrate vienen de config/servos.yaml (rutas /dev/serial/by-id/...,
# estables ante reordenamiento de ttyUSB). /joint_states se remapea a
# /arm/joint_states para no contaminar el robot_state_publisher de la base.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


JS_REMAP = [
    ('/joint_states',         '/arm/joint_states'),
    ('/joint_states_preview', '/arm/joint_states_preview'),
]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control_brazo'),
        'config', 'servos.yaml'
    )
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(
            package='control_brazo', executable='ax12a_driver',
            name='ax12a_driver', parameters=[config], remappings=JS_REMAP,
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='control_brazo', executable='ex106_driver',
            name='ex106_driver', parameters=[config], remappings=JS_REMAP,
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
