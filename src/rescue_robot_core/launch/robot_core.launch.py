from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    logitech_index = LaunchConfiguration('logitech_index')
    logitech_width = LaunchConfiguration('logitech_width')
    logitech_height = LaunchConfiguration('logitech_height')
    logitech_fps = LaunchConfiguration('logitech_fps')
    logitech_fourcc = LaunchConfiguration('logitech_fourcc')
    camera_buffer_size = LaunchConfiguration('camera_buffer_size')
    jpeg_quality = LaunchConfiguration('jpeg_quality')
    astra_depth_index = LaunchConfiguration('astra_depth_index')
    astra_color_index = LaunchConfiguration('astra_color_index')
    astra_fps = LaunchConfiguration('astra_fps')
    astra_depth_fourcc = LaunchConfiguration('astra_depth_fourcc')
    astra_color_fourcc = LaunchConfiguration('astra_color_fourcc')
    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    depth_scale = LaunchConfiguration('depth_scale')
    point_cloud_stride = LaunchConfiguration('point_cloud_stride')
    point_cloud_every_n = LaunchConfiguration('point_cloud_every_n')
    point_cloud_max_depth_m = LaunchConfiguration('point_cloud_max_depth_m')
    max_pwm = LaunchConfiguration('max_pwm')
    pwm_frequency_hz = LaunchConfiguration('pwm_frequency_hz')
    cmd_timeout_seconds = LaunchConfiguration('cmd_timeout_seconds')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        DeclareLaunchArgument('logitech_index', default_value='0'),
        DeclareLaunchArgument('logitech_width', default_value='640'),
        DeclareLaunchArgument('logitech_height', default_value='480'),
        DeclareLaunchArgument('logitech_fps', default_value='30'),
        DeclareLaunchArgument('logitech_fourcc', default_value='MJPG'),
        DeclareLaunchArgument('camera_buffer_size', default_value='1'),
        DeclareLaunchArgument('jpeg_quality', default_value='80'),
        DeclareLaunchArgument('astra_depth_index', default_value='-1'),
        DeclareLaunchArgument('astra_color_index', default_value='2'),
        DeclareLaunchArgument('astra_fps', default_value='30'),
        DeclareLaunchArgument('astra_depth_fourcc', default_value='YUYV'),
        DeclareLaunchArgument('astra_color_fourcc', default_value='MJPG'),
        DeclareLaunchArgument('fx', default_value='525.0'),
        DeclareLaunchArgument('fy', default_value='525.0'),
        DeclareLaunchArgument('cx', default_value='319.5'),
        DeclareLaunchArgument('cy', default_value='239.5'),
        DeclareLaunchArgument('depth_scale', default_value='0.001'),
        DeclareLaunchArgument('point_cloud_stride', default_value='8'),
        DeclareLaunchArgument('point_cloud_every_n', default_value='3'),
        DeclareLaunchArgument('point_cloud_max_depth_m', default_value='5.0'),
        DeclareLaunchArgument('max_pwm', default_value='0.85'),
        DeclareLaunchArgument('pwm_frequency_hz', default_value='1000'),
        DeclareLaunchArgument('cmd_timeout_seconds', default_value='1.0'),
        Node(
            package='rescue_robot_core',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
            parameters=[{
                'max_pwm': ParameterValue(max_pwm, value_type=float),
                'pwm_frequency_hz': ParameterValue(pwm_frequency_hz, value_type=int),
                'cmd_timeout_seconds': ParameterValue(cmd_timeout_seconds, value_type=float),
            }],
        ),
        Node(
            package='rescue_robot_core',
            executable='logitech_camera_node',
            name='logitech_camera_node',
            output='screen',
            parameters=[{
                'index': ParameterValue(logitech_index, value_type=int),
                'width': ParameterValue(logitech_width, value_type=int),
                'height': ParameterValue(logitech_height, value_type=int),
                'fps': ParameterValue(logitech_fps, value_type=int),
                'fourcc': logitech_fourcc,
                'buffer_size': ParameterValue(camera_buffer_size, value_type=int),
                'jpeg_quality': ParameterValue(jpeg_quality, value_type=int),
            }],
        ),
        Node(
            package='rescue_robot_core',
            executable='astra_rgbd_camera_node',
            name='astra_rgbd_camera_node',
            output='screen',
            parameters=[{
                'depth_index': ParameterValue(astra_depth_index, value_type=int),
                'color_index': ParameterValue(astra_color_index, value_type=int),
                'fps': ParameterValue(astra_fps, value_type=int),
                'depth_fourcc': astra_depth_fourcc,
                'color_fourcc': astra_color_fourcc,
                'buffer_size': ParameterValue(camera_buffer_size, value_type=int),
                'jpeg_quality': ParameterValue(jpeg_quality, value_type=int),
                'fx': ParameterValue(fx, value_type=float),
                'fy': ParameterValue(fy, value_type=float),
                'cx': ParameterValue(cx, value_type=float),
                'cy': ParameterValue(cy, value_type=float),
                'depth_scale': ParameterValue(depth_scale, value_type=float),
                'point_cloud_stride': ParameterValue(point_cloud_stride, value_type=int),
                'point_cloud_every_n': ParameterValue(point_cloud_every_n, value_type=int),
                'point_cloud_max_depth_m': ParameterValue(point_cloud_max_depth_m, value_type=float),
            }],
        ),
    ])
