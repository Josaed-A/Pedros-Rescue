from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    front_camera_topic = LaunchConfiguration('front_camera_topic')
    astra_color_topic = LaunchConfiguration('astra_color_topic')
    astra_depth_topic = LaunchConfiguration('astra_depth_topic')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    joy_autorepeat_rate = LaunchConfiguration('joy_autorepeat_rate')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    cmd_publish_rate_hz = LaunchConfiguration('cmd_publish_rate_hz')
    joy_timeout_seconds = LaunchConfiguration('joy_timeout_seconds')

    return LaunchDescription([
        DeclareLaunchArgument(
            'front_camera_topic',
            default_value='/robot/camera/front/image_raw',
        ),
        DeclareLaunchArgument(
            'astra_color_topic',
            default_value='/robot/camera/astra/color/image_raw',
        ),
        DeclareLaunchArgument(
            'astra_depth_topic',
            default_value='/robot/camera/astra/depth/image_raw',
        ),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/robot/camera/astra/points',
        ),
        DeclareLaunchArgument(
            'joy_autorepeat_rate',
            default_value='20.0',
        ),
        DeclareLaunchArgument(
            'joy_deadzone',
            default_value='0.05',
        ),
        DeclareLaunchArgument(
            'cmd_publish_rate_hz',
            default_value='20.0',
        ),
        DeclareLaunchArgument(
            'joy_timeout_seconds',
            default_value='0.7',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'autorepeat_rate': ParameterValue(joy_autorepeat_rate, value_type=float),
                'deadzone': ParameterValue(joy_deadzone, value_type=float),
            }],
        ),
        Node(
            package='rescue_command_station',
            executable='ps4_teleop_node',
            name='ps4_teleop_node',
            output='screen',
            parameters=[{
                'cmd_publish_rate_hz': ParameterValue(cmd_publish_rate_hz, value_type=float),
                'joy_timeout_seconds': ParameterValue(joy_timeout_seconds, value_type=float),
            }],
        ),
        Node(
            package='rescue_command_station',
            executable='dashboard_node',
            name='drive_dashboard_node',
            output='screen',
            parameters=[{
                'front_camera_topic': front_camera_topic,
                'astra_color_topic': astra_color_topic,
                'astra_depth_topic': astra_depth_topic,
                'point_cloud_topic': point_cloud_topic,
            }],
        ),
    ])
