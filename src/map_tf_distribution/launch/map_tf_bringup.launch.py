"""Bring up map and TF only distribution nodes."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("map_tf_distribution")
    default_params = os.path.join(pkg_share, "config", "map_tf_params.yaml")
    default_map = os.path.join(pkg_share, "maps", "occupancy_map.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "map_tf.rviz")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    map_topic = LaunchConfiguration("map_topic")
    global_frame = LaunchConfiguration("global_frame")
    child_frame = LaunchConfiguration("child_frame")
    pose_topic = LaunchConfiguration("pose_topic")
    pose_rate_hz = LaunchConfiguration("pose_rate_hz")
    tf_lookup_timeout_s = LaunchConfiguration("tf_lookup_timeout_s")
    publish_static_tf = LaunchConfiguration("publish_static_tf")
    tf_x = LaunchConfiguration("tf_x")
    tf_y = LaunchConfiguration("tf_y")
    tf_z = LaunchConfiguration("tf_z")
    tf_yaw = LaunchConfiguration("tf_yaw")
    tf_pitch = LaunchConfiguration("tf_pitch")
    tf_roll = LaunchConfiguration("tf_roll")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    map_node = Node(
        package="map_tf_distribution",
        executable="map_publisher_node",
        name="map_publisher_node",
        output="screen",
        parameters=[
            params_file,
            {
                "map_yaml": map_yaml,
                "map_topic": map_topic,
                "frame_id": global_frame,
            },
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_base_link_static",
        arguments=[
            tf_x,
            tf_y,
            tf_z,
            tf_yaw,
            tf_pitch,
            tf_roll,
            global_frame,
            child_frame,
        ],
        condition=IfCondition(publish_static_tf),
    )

    localization_tf = Node(
        package="map_tf_distribution",
        executable="localization_tf_node",
        name="localization_tf_node",
        output="screen",
        parameters=[
            params_file,
            {
                "global_frame": global_frame,
                "base_frame": child_frame,
                "pose_topic": pose_topic,
                "pose_rate_hz": pose_rate_hz,
                "tf_lookup_timeout_s": tf_lookup_timeout_s,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to map_tf_params.yaml",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value=default_map,
            description="Path to map_server-format YAML",
        ),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("global_frame", default_value="map"),
        DeclareLaunchArgument("child_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "pose_topic",
            default_value="/map_tf_distribution/localization_pose",
            description="PoseStamped topic republished from localization TF",
        ),
        DeclareLaunchArgument("pose_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("tf_lookup_timeout_s", default_value="0.05"),
        DeclareLaunchArgument(
            "publish_static_tf",
            default_value="false",
            description="Publish static global_frame->child_frame for demos",
        ),
        DeclareLaunchArgument("tf_x", default_value="0.0"),
        DeclareLaunchArgument("tf_y", default_value="0.0"),
        DeclareLaunchArgument("tf_z", default_value="0.0"),
        DeclareLaunchArgument("tf_yaw", default_value="0.0"),
        DeclareLaunchArgument("tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("tf_roll", default_value="0.0"),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz with the bundled map/TF config",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="Path to .rviz config",
        ),
        map_node,
        static_tf,
        localization_tf,
        rviz_node,
    ])
