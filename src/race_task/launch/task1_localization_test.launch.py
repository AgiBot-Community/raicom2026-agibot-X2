#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_tf_share = get_package_share_directory('map_tf_distribution')
    lidar_loc_share = get_package_share_directory('lidar_localization_ros2')

    map_tf_launch = os.path.join(
        map_tf_share,
        'launch',
        'map_tf_bringup.launch.py'
    )

    lidar_nav_launch = os.path.join(
        lidar_loc_share,
        'launch',
        'nav2_navigation.launch.py'
    )

    default_map_yaml = os.path.join(
        map_tf_share,
        'maps',
        'occupancy_map.yaml'
    )

    default_localization_params = os.path.join(
        lidar_loc_share,
        'param',
        'mid360_legged.yaml'
    )

    map_yaml = LaunchConfiguration('map_yaml')
    localization_param_dir = LaunchConfiguration('localization_param_dir')
    pcd_map_path = LaunchConfiguration('pcd_map_path')

    cloud_topic = LaunchConfiguration('cloud_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    twist_topic = LaunchConfiguration('twist_topic')

    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_qx = LaunchConfiguration('initial_pose_qx')
    initial_pose_qy = LaunchConfiguration('initial_pose_qy')
    initial_pose_qz = LaunchConfiguration('initial_pose_qz')
    initial_pose_qw = LaunchConfiguration('initial_pose_qw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml
        ),
        DeclareLaunchArgument(
            'localization_param_dir',
            default_value=default_localization_params
        ),
        DeclareLaunchArgument(
            'pcd_map_path',
            default_value=''
        ),

        # 真机上这几个话题要按实际机器人输出修改
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/velodyne_points'
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/velodyne_points'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data'
        ),
        DeclareLaunchArgument(
            'twist_topic',
            default_value='/twist'
        ),

        # 初始位姿，先给地图范围内的点
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='-20.0'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='15.0'
        ),
        DeclareLaunchArgument(
            'initial_pose_z',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'initial_pose_qx',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'initial_pose_qy',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'initial_pose_qz',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'initial_pose_qw',
            default_value='1.0'
        ),

        # 只发布 2D 地图，不发布静态 map->base_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_tf_launch),
            launch_arguments={
                'map_yaml': map_yaml,
                'rviz': 'false',
                'publish_static_tf': 'false',
            }.items()
        ),

        # 启动点云定位，但关闭 Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_nav_launch),
            launch_arguments={
                'launch_nav2': 'false',

                'localization_param_dir': localization_param_dir,
                'pcd_map_path': pcd_map_path,

                'cloud_topic': cloud_topic,
                'pointcloud_topic': pointcloud_topic,
                'imu_topic': imu_topic,
                'twist_topic': twist_topic,

                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',

                'localizer_enable_map_odom_tf': 'true',
                'localizer_enable_timer_publishing': 'true',
                'localizer_pose_publish_frequency': '20.0',

                'set_initial_pose': 'true',
                'initial_pose_x': initial_pose_x,
                'initial_pose_y': initial_pose_y,
                'initial_pose_z': initial_pose_z,
                'initial_pose_qx': initial_pose_qx,
                'initial_pose_qy': initial_pose_qy,
                'initial_pose_qz': initial_pose_qz,
                'initial_pose_qw': initial_pose_qw,

                # 不用假定位
                'use_odom_localization_demo': 'false',
                'publish_cmd_vel_odom': 'false',
                'publish_identity_odom': 'false',
                'publish_twist_odom': 'false',

                # 默认外参先保持 0，真机后按实际雷达安装位置改
                'publish_lidar_tf': 'true',
                'lidar_frame_id': 'velodyne',
                'lidar_tf_x': '0.0',
                'lidar_tf_y': '0.0',
                'lidar_tf_z': '0.0',
                'lidar_tf_roll': '0.0',
                'lidar_tf_pitch': '0.0',
                'lidar_tf_yaw': '0.0',

                'publish_imu_tf': 'true',
                'imu_frame_id': 'imu_link',
                'imu_tf_x': '0.0',
                'imu_tf_y': '0.0',
                'imu_tf_z': '0.0',
                'imu_tf_roll': '0.0',
                'imu_tf_pitch': '0.0',
                'imu_tf_yaw': '0.0',
            }.items()
        ),
    ])
