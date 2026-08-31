import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'x2_execution_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='magic',
    maintainer_email='magic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_node = x2_execution_pkg.voice_node:main',
            'vision_node = x2_execution_pkg.vision_node:main',
            'coordinator_node = x2_execution_pkg.coordinator_node:main',
            'execution_node = x2_execution_pkg.execution_dispatcher:main',
        ],
    },

)
