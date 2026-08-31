from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    map_tf_share = get_package_share_directory('map_tf_distribution')
    race_task_share = get_package_share_directory('race_task')

    map_tf_launch = os.path.join(
        map_tf_share,
        'launch',
        'map_tf_bringup.launch.py'
    )

    task1_params = os.path.join(
        race_task_share,
        'config',
        'task1_params.yaml'
    )

    # 实机时 publish_static_tf 应该是 false
    publish_static_tf = LaunchConfiguration('publish_static_tf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_static_tf',
            default_value='false',
            description='Use true only for offline RViz test without robot.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_tf_launch),
            launch_arguments={
                'publish_static_tf': publish_static_tf,
                'rviz': 'false',
            }.items()
        ),

        Node(
            package='race_task',
            executable='task1_auto',
            name='task1_auto',
            output='screen',
            parameters=[task1_params],
        ),
    ])
