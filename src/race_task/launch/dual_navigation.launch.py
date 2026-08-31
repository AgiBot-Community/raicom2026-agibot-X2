from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task1_params = LaunchConfiguration('task1_params')
    service_params = LaunchConfiguration('service_params')

    default_task1 = PathJoinSubstitution([
        FindPackageShare('race_task'), 'config', 'task1_app_ready.yaml'
    ])
    default_service = PathJoinSubstitution([
        FindPackageShare('race_task'), 'config', 'service_official_map_nav.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('task1_params', default_value=default_task1),
        DeclareLaunchArgument('service_params', default_value=default_service),

        # Localization pose bridge: only one instance.
        Node(
            package='race_task',
            executable='tf_pose_bridge',
            name='tf_pose_bridge',
            output='screen',
            parameters=[task1_params],
        ),

        # Physical velocity bridge: only one instance.
        Node(
            package='race_task',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[task1_params],
        ),

        Node(
            package='race_task',
            executable='task1_pose_nav',
            name='task1_pose_nav',
            output='screen',
            parameters=[task1_params],
        ),

        Node(
            package='race_task',
            executable='service_pose_nav',
            name='service_pose_nav',
            output='screen',
            parameters=[service_params],
        ),
    ])
