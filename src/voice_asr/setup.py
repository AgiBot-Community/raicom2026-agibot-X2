from glob import glob
from setuptools import find_packages, setup

package_name = "voice_asr"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="star",
    maintainer_email="star@todo.todo",
    description=(
        "Offline SenseVoice ASR with three robot audio pipelines: "
        "internal raw + Silero, external raw + Silero, and "
        "external processed + AimDK VAD."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "voice_asr_node = voice_asr.voice_asr_node:main",
            (
                "voice_asr_raw_internal = "
                "voice_asr.voice_asr_node:main_raw_internal"
            ),
            (
                "voice_asr_raw_external = "
                "voice_asr.voice_asr_node:main_raw_external"
            ),
            (
                "voice_asr_processed_external = "
                "voice_asr.voice_asr_node:main_processed_external"
            ),
        ],
    },
)
