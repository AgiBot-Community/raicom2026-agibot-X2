#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    race_task_share = get_package_share_directory('race_task')
    lidar_loc_share = get_package_share_directory('lidar_localization_ros2')
    map_tf_share = get_package_share_directory('map_tf_distribution')

    nav2_launch = os.path.join(
        lidar_loc_share,
        'launch',
        'nav2_navigation.launch.py'
    )

    default_map_yaml = os.path.join(
        map_tf_share,
        'maps',
        'occupancy_map.yaml'
    )

    map_yaml = LaunchConfiguration('map_yaml')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_yaw = LaunchConfiguration('target_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml
        ),
        DeclareLaunchArgument(
            'target_x',
            default_value='1.0'
        ),
        DeclareLaunchArgument(
            'target_y',
            default_value='2.0'
        ),
        DeclareLaunchArgument(
            'target_yaw',
            default_value='0.0'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map_yaml': map_yaml,

                # 离线测试用：不用真雷达定位，先用 /cmd_vel 积分出 odom
                'use_odom_localization_demo': 'true',
                'publish_cmd_vel_odom': 'true',

                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'odom_topic': '/odom',

                # 初始位姿，先放在 map 坐标系原点附近
                'initial_pose_x': '-20.0',
                'initial_pose_y': '15.0',
                'initial_pose_z': '0.0',
                'initial_pose_qx': '0.0',
                'initial_pose_qy': '0.0',
                'initial_pose_qz': '0.0',
                'initial_pose_qw': '1.0',
            }.items()
        ),

        Node(
            package='race_task',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'register_input_source': False,
                'source': 'node',
                'cmd_vel_topic': '/cmd_vel',
                'output_topic': '/aima/mc/locomotion/velocity',
                'max_forward_velocity': 0.4,
                'max_backward_velocity': 0.2,
                'max_lateral_velocity': 0.0,
                'max_angular_velocity': 0.5,
                'cmd_timeout_sec': 0.5,
                'publish_hz': 20.0,
            }]
        ),

        # 等 Nav2 启动后再发送目标点
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='race_task',
                    executable='goal_sender',
                    name='goal_sender',
                    output='screen',
                    parameters=[{
                        'frame_id': 'map',
                        'target_x': target_x,
                        'target_y': target_y,
                        'target_z': 0.0,
                        'target_yaw': target_yaw,
                        'wait_server_timeout_sec': 90.0,
                        'result_timeout_sec': 300.0,
                    }]
                )
            ]
        ),
    ])
