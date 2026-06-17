"""
High-level hardware bringup for the Raspberry Pi.

Runs the Pi side in one launch:
  - ROS 2 network environment for CycloneDDS.
  - Robot description and TF.
  - LD19 lidar.
  - Astra/Logitech cameras and detector.
  - BTS7960 GPIO/PWM motor driver.
  - Dynamixel AX-12A/EX-106+ drivers for arm and legs.

Usage:
  ros2 launch rescue_bringup pedro_pi.launch.py
"""

import os
import glob
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def _usb_device_present(vendor_id: str, product_id: str | None = None) -> bool:
    for dev in glob.glob('/sys/bus/usb/devices/*'):
        try:
            with open(os.path.join(dev, 'idVendor'), encoding='utf-8') as f:
                vendor = f.read().strip().lower()
            with open(os.path.join(dev, 'idProduct'), encoding='utf-8') as f:
                product = f.read().strip().lower()
        except OSError:
            continue
        if vendor == vendor_id.lower() and (product_id is None or product == product_id.lower()):
            return True
    return False


def _video_props(device: str) -> dict[str, str]:
    try:
        result = subprocess.run(
            ['udevadm', 'info', '-q', 'property', '-n', device],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
    except OSError:
        return {}

    props = {}
    for line in result.stdout.splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            props[key] = value
    return props


def _video_index_for_usb(vendor_id: str | None = None, exclude_vendor_id: str | None = None):
    for device in sorted(glob.glob('/dev/video*')):
        props = _video_props(device)
        if props.get('ID_BUS') != 'usb':
            continue
        vendor = props.get('ID_VENDOR_ID', '').lower()
        if vendor_id is not None and vendor != vendor_id.lower():
            continue
        if exclude_vendor_id is not None and vendor == exclude_vendor_id.lower():
            continue
        try:
            return int(device.removeprefix('/dev/video'))
        except ValueError:
            continue
    return None


def _resolve_auto_bool(value: str, detected: bool) -> str:
    if value.lower() == 'auto':
        return 'true' if detected else 'false'
    return value


def _configure_network(context):
    network = LaunchConfiguration('network').perform(context).lower()
    cable_iface = LaunchConfiguration('cable_interface').perform(context)
    wifi_iface = LaunchConfiguration('wifi_interface').perform(context)
    pc_cable_ip = LaunchConfiguration('pc_cable_ip').perform(context)
    pc_wifi_ip = LaunchConfiguration('pc_wifi_ip').perform(context)
    pi_cable_ip = LaunchConfiguration('pi_cable_ip').perform(context)
    pi_wifi_ip = LaunchConfiguration('pi_wifi_ip').perform(context)

    if network == 'auto':
        network = 'cable' if _ping_ok(pc_cable_ip) else 'wifi'

    if network == 'wifi':
        interface = wifi_iface
        peer = pc_wifi_ip
        self_ip = pi_wifi_ip
    else:
        network = 'cable'
        interface = cable_iface
        peer = pc_cable_ip
        self_ip = pi_cable_ip

    uri = (
        '<CycloneDDS><Domain><General><Interfaces>'
        f'<NetworkInterface name="{interface}" multicast="false"/>'
        '</Interfaces></General><Discovery><Peers>'
        f'<Peer Address="{peer}"/>'
        f'<Peer Address="{self_ip}"/>'
        '</Peers></Discovery></Domain></CycloneDDS>'
    )

    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
        SetEnvironmentVariable('ROS_DISCOVERY_SERVER', ''),
        SetEnvironmentVariable('CYCLONEDDS_URI', uri),
        LogInfo(msg=f'[red Pi] CycloneDDS por {network} ({interface}) -> PC {peer}'),
    ]


def _launch_sensors(context):
    pkg_bringup = get_package_share_directory('rescue_bringup')

    launch_camera_arg = LaunchConfiguration('launch_camera').perform(context)
    launch_logitech_arg = LaunchConfiguration('launch_logitech').perform(context)
    astra_color_arg = LaunchConfiguration('astra_color_index').perform(context)
    logitech_device_arg = LaunchConfiguration('logitech_device').perform(context)

    orbbec_present = _usb_device_present('2bc5')
    orbbec_video = _video_index_for_usb(vendor_id='2bc5')
    logitech_video = _video_index_for_usb(vendor_id='046d')
    if logitech_video is None:
        logitech_video = _video_index_for_usb(exclude_vendor_id='2bc5')

    launch_camera = _resolve_auto_bool(launch_camera_arg, orbbec_present or orbbec_video is not None)
    launch_logitech = _resolve_auto_bool(launch_logitech_arg, logitech_video is not None)

    astra_color_index = astra_color_arg
    if astra_color_arg.lower() == 'auto':
        astra_color_index = str(orbbec_video if orbbec_video is not None else 2)

    logitech_device = logitech_device_arg
    if logitech_device_arg.lower() == 'auto':
        logitech_device = str(logitech_video if logitech_video is not None else 0)

    return [
        LogInfo(
            msg=(
                '[hardware Pi] '
                f'Orbbec={"detectada" if launch_camera == "true" else "no detectada"} '
                f'(color_index={astra_color_index}); '
                f'Logitech={"detectada" if launch_logitech == "true" else "no detectada"} '
                f'(device={logitech_device})'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'pi_sensors.launch.py')
            ),
            launch_arguments={
                'launch_robot_description': LaunchConfiguration('launch_robot_description'),
                'launch_lidar': LaunchConfiguration('launch_lidar'),
                'launch_camera': launch_camera,
                'launch_logitech': launch_logitech,
                'camera_driver': LaunchConfiguration('camera_driver'),
                'astra_depth_index': LaunchConfiguration('astra_depth_index'),
                'astra_color_index': astra_color_index,
                'astra_fps': LaunchConfiguration('astra_fps'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'logitech_device': logitech_device,
                'hazmat_model': LaunchConfiguration('hazmat_model'),
                'output_dir': LaunchConfiguration('output_dir'),
            }.items(),
        ),
    ]


def generate_launch_description():
    pkg_core = get_package_share_directory('rescue_robot_core')

    launch_motors = LaunchConfiguration('launch_motors')
    launch_servos = LaunchConfiguration('launch_servos')
    max_pwm = LaunchConfiguration('max_pwm')
    pwm_frequency_hz = LaunchConfiguration('pwm_frequency_hz')
    cmd_timeout_seconds = LaunchConfiguration('cmd_timeout_seconds')

    motors_node = Node(
        package='rescue_robot_core',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen',
        condition=IfCondition(launch_motors),
        parameters=[{
            'max_pwm': ParameterValue(max_pwm, value_type=float),
            'pwm_frequency_hz': ParameterValue(pwm_frequency_hz, value_type=int),
            'cmd_timeout_seconds': ParameterValue(cmd_timeout_seconds, value_type=float),
        }],
    )

    servos_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_core, 'launch', 'servos.launch.py')
        ),
        condition=IfCondition(launch_servos),
    )

    return LaunchDescription([
        DeclareLaunchArgument('network', default_value='auto',
                              description='auto | cable | wifi'),
        DeclareLaunchArgument('cable_interface', default_value='eth0'),
        DeclareLaunchArgument('wifi_interface', default_value='wlan0'),
        DeclareLaunchArgument('pc_cable_ip', default_value='10.42.0.1'),
        DeclareLaunchArgument('pc_wifi_ip', default_value='192.168.231.15'),
        DeclareLaunchArgument('pi_cable_ip', default_value='10.42.0.240'),
        DeclareLaunchArgument('pi_wifi_ip', default_value='192.168.231.137'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='auto',
                              description='auto | true | false'),
        DeclareLaunchArgument('launch_logitech', default_value='auto',
                              description='auto | true | false'),
        DeclareLaunchArgument('launch_robot_description', default_value='true'),
        DeclareLaunchArgument('launch_motors', default_value='true'),
        DeclareLaunchArgument('launch_servos', default_value='true'),
        DeclareLaunchArgument('camera_driver', default_value='astra_core'),
        DeclareLaunchArgument('astra_depth_index', default_value='-1'),
        DeclareLaunchArgument('astra_color_index', default_value='auto'),
        DeclareLaunchArgument('astra_fps', default_value='30'),
        DeclareLaunchArgument('jpeg_quality', default_value='80'),
        DeclareLaunchArgument('logitech_device', default_value='auto'),
        DeclareLaunchArgument('hazmat_model', default_value=''),
        DeclareLaunchArgument('output_dir', default_value='/home/gardian/maps'),
        DeclareLaunchArgument('max_pwm', default_value='0.85'),
        DeclareLaunchArgument('pwm_frequency_hz', default_value='1000'),
        DeclareLaunchArgument('cmd_timeout_seconds', default_value='1.0'),
        OpaqueFunction(function=_configure_network),
        OpaqueFunction(function=_launch_sensors),
        motors_node,
        servos_launch,
    ])
