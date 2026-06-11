from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    logitech_index = LaunchConfiguration('logitech_index')
    logitech_width = LaunchConfiguration('logitech_width')
    logitech_height = LaunchConfiguration('logitech_height')
    logitech_fps = LaunchConfiguration('logitech_fps')
    astra_depth_index = LaunchConfiguration('astra_depth_index')
    astra_color_index = LaunchConfiguration('astra_color_index')
    astra_fps = LaunchConfiguration('astra_fps')
    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    depth_scale = LaunchConfiguration('depth_scale')
    point_cloud_stride = LaunchConfiguration('point_cloud_stride')
    point_cloud_max_depth_m = LaunchConfiguration('point_cloud_max_depth_m')

    return LaunchDescription([
        DeclareLaunchArgument('logitech_index', default_value='0'),
        DeclareLaunchArgument('logitech_width', default_value='640'),
        DeclareLaunchArgument('logitech_height', default_value='480'),
        DeclareLaunchArgument('logitech_fps', default_value='30'),
        DeclareLaunchArgument('astra_depth_index', default_value='2'),
        DeclareLaunchArgument('astra_color_index', default_value='3'),
        DeclareLaunchArgument('astra_fps', default_value='30'),
        DeclareLaunchArgument('fx', default_value='525.0'),
        DeclareLaunchArgument('fy', default_value='525.0'),
        DeclareLaunchArgument('cx', default_value='319.5'),
        DeclareLaunchArgument('cy', default_value='239.5'),
        DeclareLaunchArgument('depth_scale', default_value='0.001'),
        DeclareLaunchArgument('point_cloud_stride', default_value='4'),
        DeclareLaunchArgument('point_cloud_max_depth_m', default_value='5.0'),
        Node(
            package='rescue_robot_core',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
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
                'fx': ParameterValue(fx, value_type=float),
                'fy': ParameterValue(fy, value_type=float),
                'cx': ParameterValue(cx, value_type=float),
                'cy': ParameterValue(cy, value_type=float),
                'depth_scale': ParameterValue(depth_scale, value_type=float),
                'point_cloud_stride': ParameterValue(point_cloud_stride, value_type=int),
                'point_cloud_max_depth_m': ParameterValue(point_cloud_max_depth_m, value_type=float),
            }],
        ),
    ])
