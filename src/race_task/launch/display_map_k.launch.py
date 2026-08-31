import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 指向你具体的静态地图文件绝对路径
    map_file_path = '/agibot/data/var/MapManagerModule/1786509067171/occupancy_map.yaml'

    return LaunchDescription([
        # 1. 启动 Map Server，负责读取 YAML 和 PGM 文件并发布到 /map 话题
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}]
        ),
        
        # 2. 启动 Lifecycle Manager (Nav2 的核心节点必须通过生命周期管理器激活)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server']} # 告知管理器需要激活哪些节点
            ]
        ),

        # 3. 启动 RViz2 界面
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])