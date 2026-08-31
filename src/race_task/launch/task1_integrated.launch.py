from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    start_bridge = LaunchConfiguration("start_bridge")
    start_tf_pose_bridge = LaunchConfiguration("start_tf_pose_bridge")

    default_params = PathJoinSubstitution([
        FindPackageShare("race_task"),
        "config",
        "task1_integrated.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Task1 integrated params file",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start cmd_vel_bridge",
        ),
        DeclareLaunchArgument(
            "start_tf_pose_bridge",
            default_value="true",
            description="Start tf_pose_bridge",
        ),
        Node(
            package="race_task",
            executable="tf_pose_bridge",
            name="tf_pose_bridge",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(start_tf_pose_bridge),
        ),
        Node(
            package="race_task",
            executable="cmd_vel_bridge",
            name="cmd_vel_bridge",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(start_bridge),
        ),
        Node(
            package="race_task",
            executable="task1_pose_nav",
            name="task1_pose_nav",
            output="screen",
            parameters=[params_file],
        ),
    ])
