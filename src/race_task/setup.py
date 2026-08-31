from setuptools import setup
import os
from glob import glob

package_name = 'race_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='star@example.com',
    description='Race task package for AGI robot competition task 1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task1_auto = race_task.task1_auto:main',
            'cmd_vel_bridge = race_task.cmd_vel_bridge:main',
            'goal_sender = race_task.goal_sender:main',
            'odom_visualizer = race_task.odom_visualizer:main',
            'robot_odom_bridge = race_task.robot_odom_bridge:main',
            'task1_pose_nav = race_task.task1_pose_nav:main',
            'service_pose_nav = race_task.service_pose_nav:main',
            'goal_capture = race_task.goal_capture:main',
            'fake_localization_pose = race_task.fake_localization_pose:main',
            'tf_pose_bridge = race_task.tf_pose_bridge:main',
        ],
    },
)
