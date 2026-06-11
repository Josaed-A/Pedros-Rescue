from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    front_camera_topic = LaunchConfiguration('front_camera_topic')
    astra_color_topic = LaunchConfiguration('astra_color_topic')
    astra_depth_topic = LaunchConfiguration('astra_depth_topic')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')

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
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='rescue_command_station',
            executable='ps4_teleop_node',
            name='ps4_teleop_node',
            output='screen',
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
