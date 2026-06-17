"""
High-level command station bringup for the PC.

Runs the PC side in one launch:
  - ROS 2 network environment for CycloneDDS.
  - SLAM and RViz, consuming lidar/camera topics from the Pi.
  - Dashboard, joy_node, and PS4 teleop.

Usage:
  ros2 launch rescue_bringup pedro_pc.launch.py
"""

import glob
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def _ping_ok(host: str) -> bool:
    try:
        return subprocess.run(
            ['ping', '-c', '1', '-W', '1', host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode == 0
    except OSError:
        return False


def _configure_network(context):
    network = LaunchConfiguration('network').perform(context).lower()
    cable_iface = LaunchConfiguration('cable_interface').perform(context)
    wifi_iface = LaunchConfiguration('wifi_interface').perform(context)
    pi_cable_ip = LaunchConfiguration('pi_cable_ip').perform(context)
    pi_wifi_ip = LaunchConfiguration('pi_wifi_ip').perform(context)
    pc_cable_ip = LaunchConfiguration('pc_cable_ip').perform(context)
    pc_wifi_ip = LaunchConfiguration('pc_wifi_ip').perform(context)

    if network == 'auto':
        network = 'cable' if _ping_ok(pi_cable_ip) else 'wifi'

    if network == 'wifi':
        interface = wifi_iface
        peer = pi_wifi_ip
        self_ip = pc_wifi_ip
    else:
        network = 'cable'
        interface = cable_iface
        peer = pi_cable_ip
        self_ip = pc_cable_ip

    uri = (
        '<CycloneDDS><Domain><General><Interfaces>'
        f'<NetworkInterface name="{interface}" multicast="false"/>'
        '</Interfaces></General><Discovery><Peers>'
        f'<Peer Address="{peer}"/>'
        f'<Peer Address="{self_ip}"/>'
        '</Peers></Discovery></Domain></CycloneDDS>'
    )
    env_actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
        SetEnvironmentVariable('ROS_DISCOVERY_SERVER', ''),
        SetEnvironmentVariable('CYCLONEDDS_URI', uri),
    ]

    vendor_lib_paths = []
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        vendor_lib_paths.extend(
            path for path in glob.glob(os.path.join(prefix, 'opt', '*', 'lib'))
            if os.path.isdir(path)
        )
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    env_actions.append(
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            ':'.join(path for path in [*vendor_lib_paths, ld_library_path] if path),
        )
    )

    env_actions.append(
        LogInfo(msg=f'[red PC] CycloneDDS por {network} ({interface}) -> Pi {peer}'),
    )
    return env_actions


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rescue_bringup')
    pkg_station = get_package_share_directory('rescue_command_station')

    launch_slam = LaunchConfiguration('launch_slam')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_dashboard = LaunchConfiguration('launch_dashboard')
    launch_detector = LaunchConfiguration('launch_detector')
    output_dir = LaunchConfiguration('output_dir')
    hazmat_model = LaunchConfiguration('hazmat_model')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(launch_slam),
        launch_arguments={
            'launch_robot_description': 'false',
            'launch_lidar': 'false',
            'launch_camera': 'false',
            'launch_rviz': launch_rviz,
            'launch_detector': launch_detector,
            'hazmat_model': hazmat_model,
            'output_dir': output_dir,
        }.items(),
    )

    station_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_station, 'launch', 'command_station.launch.py')
        ),
        condition=IfCondition(launch_dashboard),
    )

    return LaunchDescription([
        DeclareLaunchArgument('network', default_value='auto',
                              description='auto | cable | wifi'),
        DeclareLaunchArgument('cable_interface', default_value='eno1'),
        DeclareLaunchArgument('wifi_interface', default_value='wlp0s20f3'),
        DeclareLaunchArgument('pi_cable_ip', default_value='10.42.0.240'),
        DeclareLaunchArgument('pi_wifi_ip', default_value='192.168.231.137'),
        DeclareLaunchArgument('pc_cable_ip', default_value='10.42.0.1'),
        DeclareLaunchArgument('pc_wifi_ip', default_value='192.168.231.15'),
        DeclareLaunchArgument('launch_slam', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('launch_dashboard', default_value='true'),
        DeclareLaunchArgument('launch_detector', default_value='false'),
        DeclareLaunchArgument('hazmat_model', default_value=''),
        DeclareLaunchArgument(
            'output_dir',
            default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'maps']),
        ),
        OpaqueFunction(function=_configure_network),
        slam_launch,
        station_launch,
    ])
