#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-$HOME/aimdk}"

echo "========== Task2: Build and Run play_linkcraft =========="

cd "$WORKSPACE"

echo "[1/4] Source ROS Humble"
source /opt/ros/humble/setup.bash

echo "[2/4] Build aimdk"
colcon build

echo "[3/4] Source aimdk environment"
source install/local_setup.bash

echo "[4/4] Run play_linkcraft"
echo

exec ros2 run py_examples play_linkcraft
