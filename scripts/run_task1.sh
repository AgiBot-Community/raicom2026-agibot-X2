#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-$HOME/star_agibot}"
PARAMS_FILE="${PARAMS_FILE:-$WORKSPACE/src/race_task/config/task1_app_ready.yaml}"

echo "========== Run task1 app ready =========="
cd "$WORKSPACE"

echo "[1/3] Source ROS Humble"
source /opt/ros/humble/setup.bash

echo "[2/3] Source workspace"
source install/setup.bash

if [ ! -f "$PARAMS_FILE" ]; then
  echo "❌ params_file not found:"
  echo "  $PARAMS_FILE"
  exit 1
fi

echo "[3/3] Launch task1"
echo "Params file: $PARAMS_FILE"
echo

exec ros2 launch race_task task1_app_ready.launch.py \
  params_file:="$PARAMS_FILE"
