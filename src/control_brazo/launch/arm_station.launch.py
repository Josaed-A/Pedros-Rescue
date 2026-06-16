# arm_station.launch.py — lado PC (estación) del brazo 6-DOF para Pedro's Rescue.
#
# Lanza la CINEMATICA + CONTROL CARTESIANO + GUI del brazo. Los drivers reales
# de los Dynamixel corren en la Raspberry (ver arm_pi.launch.py).
#
#   ros2 launch control_brazo arm_station.launch.py            # con Pi real
#   ros2 launch control_brazo arm_station.launch.py sim:=true  # sin Pi (drivers simulados en el PC)
#
# Topics del brazo que colisionan con la base de conduccion (/joint_states,
# consumido por robot_state_publisher) se remapean a /arm/* para aislar el brazo.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Solo /joint_states(+_preview) chocan con la base; el resto de topics del
# brazo (/end_effector_pose, /fk_points, /ax12a/*, /cartesian/*, /compute_ik*)
# son exclusivos y no necesitan remapeo.
JS_REMAP = [
    ('/joint_states',         '/arm/joint_states'),
    ('/joint_states_preview', '/arm/joint_states_preview'),
]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control_brazo'),
        'config', 'servos.yaml'
    )
    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='true = drivers simulados en el PC (sin Raspberry).'),

        # ── Drivers SIMULADOS (solo si sim:=true; para probar sin la Pi) ──
        Node(
            package='control_brazo', executable='ax12a_sim_driver',
            name='ax12a_driver', parameters=[config, {'driver_ns': 'ax12a'}],
            remappings=JS_REMAP, condition=IfCondition(sim),
        ),
        Node(
            package='control_brazo', executable='ex106_sim_driver',
            name='ex106_driver', parameters=[config, {'driver_ns': 'ex106'}],
            remappings=JS_REMAP, condition=IfCondition(sim),
        ),

        # ── Cinematica (FK/IK) ──
        Node(
            package='control_brazo', executable='cinematica',
            name='cinematica', parameters=[config], remappings=JS_REMAP,
        ),
        # ── Control cartesiano (trayectorias) ──
        Node(
            package='control_brazo', executable='cartesian',
            name='cartesian', parameters=[config],
        ),
        # ── GUI del brazo (vista 3D + teleop + IK) ──
        Node(
            package='control_brazo', executable='gui_control',
            name='gui_control', parameters=[config], remappings=JS_REMAP,
        ),
    ])
