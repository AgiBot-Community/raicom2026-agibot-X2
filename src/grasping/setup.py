from setuptools import find_packages, setup

package_name = 'grasping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkk',
    maintainer_email='sheidonga@',
    description='Grasping control package for X2 Robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'omnipicker_hand = grasping.omnipicker_hand:main',
            'x2_grasp_executor_server = grasping.x2_grasp_executor_server:main',
            'x2_grasp_auto_trigger = grasping.x2_grasp_auto_trigger:main',
            'x2_grasp_worker = grasping.x2_grasp_worker:main',
        ],
    },
)