# servos.launch.py — bus de servos del robot (corre en la Raspberry Pi).
#
# Levanta el driver Dynamixel del bus AX-12A (brazo + PATAS, dynamixel_bus_node)
# y el driver EX-106+ (hombro). La cinematica/GUI del brazo corren en el PC
# (ver rescue_command_station/launch/arm_station.launch.py); las patas se controlan desde
# el dashboard (rescue_command_station).
#
#   ros2 launch rescue_robot_core servos.launch.py
#
# /joint_states (brazo) se remapea a /arm/joint_states para no contaminar el
# robot_state_publisher de la base. Las patas usan /legs/* (sin remapeo).

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
        get_package_share_directory('rescue_robot_core'), 'config', 'servos.yaml')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(
            package='rescue_robot_core', executable='dynamixel_bus_node',
            name='ax12a_driver', parameters=[config], remappings=JS_REMAP,
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='rescue_robot_core', executable='ex106_driver_node',
            name='ex106_driver', parameters=[config], remappings=JS_REMAP,
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
