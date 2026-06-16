# Lanza simulacion completa sin hardware real.
# Incluye drivers simulados que exponen los mismos topics y servicios
# que los drivers reales, de modo que la GUI funciona igual que en produccion.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control_brazo'),
        'config', 'servos.yaml'
    )

    return LaunchDescription([
        # Driver simulado AX-12A — mismos topics/servicios que el driver real
        Node(
            package='control_brazo',
            executable='ax12a_sim_driver',
            name='ax12a_driver',
            parameters=[config, {'driver_ns': 'ax12a'}],
        ),
        # Driver simulado EX-106+ — mismos topics/servicios que el driver real
        Node(
            package='control_brazo',
            executable='ex106_sim_driver',
            name='ex106_driver',
            parameters=[config, {'driver_ns': 'ex106'}],
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
