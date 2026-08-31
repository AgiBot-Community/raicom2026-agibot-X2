from glob import glob
import os

from setuptools import setup

package_name = "map_tf_distribution"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"),
         glob("maps/*.yaml") + glob("maps/*.png") + glob("maps/*.pgm")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Map and TF only ROS 2 distribution package",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "map_publisher_node = map_tf_distribution.map_publisher_node:main",
            "localization_tf_node = map_tf_distribution.localization_tf_node:main",
        ],
    },
)
