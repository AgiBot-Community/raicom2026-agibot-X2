from setuptools import find_packages, setup


package_name = "interaction_agent"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(
        exclude=("test",),
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="star",
    maintainer_email="star@example.com",
    description=(
        "Closed-set intent inference and interaction "
        "state machine for the robot task."
    ),
    license="Apache-2.0",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            (
                "interaction_node = "
                "interaction_agent.interaction_node:main"
            ),
        ],
    },
)
