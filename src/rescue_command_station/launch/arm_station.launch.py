# arm_station.launch.py — lado PC del brazo 6-DOF.
#
# Levanta CINEMATICA + CONTROL CARTESIANO + GUI del brazo. Los drivers reales
# de los servos corren en la Raspberry (rescue_robot_core/launch/servos.launch.py).
#
#   ros2 launch rescue_command_station arm_station.launch.py            # con Pi real
#   ros2 launch rescue_command_station arm_station.launch.py sim:=true  # sin Pi (drivers simulados)
#
# /joint_states del brazo se remapea a /arm/* para aislar el brazo de la base.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


JS_REMAP = [
    ('/joint_states',         '/arm/joint_states'),
    ('/joint_states_preview', '/arm/joint_states_preview'),
]


def generate_launch_description():
    arm_cfg = os.path.join(
        get_package_share_directory('rescue_command_station'), 'config', 'arm.yaml')
    servos_cfg = os.path.join(
        get_package_share_directory('rescue_robot_core'), 'config', 'servos.yaml')
    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='true = drivers simulados en el PC (sin Raspberry).'),

        # ── Drivers SIMULADOS (solo si sim:=true) — viven en rescue_robot_core ──
        Node(
            package='rescue_robot_core', executable='dynamixel_sim_node',
            name='ax12a_driver', parameters=[servos_cfg, {'driver_ns': 'ax12a'}],
            remappings=JS_REMAP, condition=IfCondition(sim),
        ),
        Node(
            package='rescue_robot_core', executable='dynamixel_sim_node',
            name='ex106_driver', parameters=[servos_cfg, {'driver_ns': 'ex106'}],
            remappings=JS_REMAP, condition=IfCondition(sim),
        ),

        # ── Brazo (PC) ──
        Node(
            package='rescue_command_station', executable='cinematica_node',
            name='cinematica', parameters=[arm_cfg], remappings=JS_REMAP,
        ),
        Node(
            package='rescue_command_station', executable='cartesian_node',
            name='cartesian', parameters=[arm_cfg],
        ),
        Node(
            package='rescue_command_station', executable='arm_gui_node',
            name='gui_control', parameters=[arm_cfg], remappings=JS_REMAP,
        ),
    ])
