from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control_brazo'),
        'config', 'servos.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        # Pasa enable_cartesian:=true para activar el nodo de trayectorias
        DeclareLaunchArgument('enable_cartesian', default_value='true'),

        Node(
            package='control_brazo',
            executable='ax12a_driver',
            name='ax12a_driver',
            parameters=[config],
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
        ),
        Node(
            package='control_brazo',
            executable='ex106_driver',
            name='ex106_driver',
            parameters=[config],
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
        ),
        Node(
            package='control_brazo',
            executable='cinematica',
            name='cinematica',
            parameters=[config],
        ),
        Node(
            package='control_brazo',
            executable='cartesian',
            name='cartesian',
            parameters=[config],
        ),
        Node(
            package='control_brazo',
            executable='gui_control',
            name='gui_control',
            parameters=[config],
        ),
    ])
