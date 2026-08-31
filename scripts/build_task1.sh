#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-$HOME/star_agibot}"

echo "========== Build task1 packages =========="
cd "$WORKSPACE"

echo "[1/5] Source ROS Humble"
source /opt/ros/humble/setup.bash

echo "[2/5] Build aimdk_msgs"
colcon build --packages-select aimdk_msgs --symlink-install

echo "[3/5] Source workspace"
source install/setup.bash

echo "[4/5] Build race_task"
colcon build --packages-select race_task --symlink-install

echo "[5/5] Source workspace again"
source install/setup.bash

echo
echo "========== race_task executables =========="
ros2 pkg executables race_task || true

echo
echo "✅ Build finished."
echo "Next:"
echo "  bash scripts/check_task1_topics.sh"
echo "  bash scripts/run_task1.sh"
